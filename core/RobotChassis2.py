import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import (
    Pose, PoseWithCovarianceStamped, Point, Quaternion,
    PointStamped, PoseStamped
)
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Nav2 actions
from nav2_msgs.action import NavigateToPose, FollowWaypoints, NavigateThroughPoses


# QoS that matches AMCL / RViz latched-style topics
AMCL_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class RobotChassis2(Node):
    # status_code values
    STATUS_PENDING   = 0
    STATUS_ACTIVE    = 1
    STATUS_SUCCEEDED = 3
    STATUS_ABORTED   = 4

    # Constructor:
    def __init__(self, frame_id="map", node_name="robot_chassis",
                 amcl_wait_timeout=10.0):
        super().__init__(node_name)

        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("Connected to navigate_to_pose server.")

        # Instance variables
        self.frame_id = frame_id
        self.initial_pose = PoseWithCovarianceStamped()
        self.current_pose = PoseWithCovarianceStamped()
        self.last_clicked_point = PointStamped()
        self.last_goal_pose = PoseStamped()

        # Flags to know whether the corresponding callbacks fired at least once
        self._got_amcl_pose = False
        self._got_initial_pose = False
        self._got_clicked_point = False
        self._got_goal_pose = False

        self.status_code = 0
        self.status_text = ""

        self._goal_handle = None
        self._result_future = None

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose",
            self.initialpose_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            self.amcl_pose_callback, AMCL_QOS
        )
        self.create_subscription(
            PointStamped, "/clicked_point",
            self.clicked_point_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/goal_pose",
            self.goal_callback, 10
        )

        # Service clients to clear costmaps (Nav2 names)
        self.clear_costmaps_global = self.create_client(
            Empty, "/global_costmap/clear_entirely_global_costmap"
        )
        self.clear_costmaps_local = self.create_client(
            Empty, "/local_costmap/clear_entirely_local_costmap"
        )

        # Wait for the first /amcl_pose so get_current_pose() works
        # immediately after the constructor returns.
        self._wait_for_amcl_pose(timeout=amcl_wait_timeout)

        self.get_logger().info("RobotChassis2 OK")

    # -------------------- helpers --------------------
    def _spin_once(self, timeout=0.1):
        rclpy.spin_once(self, timeout_sec=timeout)

    def _wait_for_amcl_pose(self, timeout=10.0):
        """Spin the node until /amcl_pose arrives (or timeout)."""
        self.get_logger().info("Waiting for first /amcl_pose message...")
        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if self._got_amcl_pose:
                p = self.current_pose.pose.pose.position
                self.get_logger().info(
                    "Got initial /amcl_pose: (%.2f, %.2f)" % (p.x, p.y)
                )
                return True
            if (time.time() - start) > timeout:
                self.get_logger().warn(
                    "Timed out waiting for /amcl_pose. Is AMCL running and "
                    "localized? get_current_pose() will return zeros until a "
                    "message arrives."
                )
                return False
        return False

    @staticmethod
    def point_to_pose(x, y, theta):
        q = quaternion_from_euler(0.0, 0.0, theta)
        return Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )

    def clear_costmaps(self):
        try:
            if self.clear_costmaps_global.wait_for_service(timeout_sec=1.0):
                self.clear_costmaps_global.call_async(Empty.Request())
            if self.clear_costmaps_local.wait_for_service(timeout_sec=1.0):
                self.clear_costmaps_local.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().warn(f"clear_costmaps failed: {e}")

    # -------------------- RViz interaction --------------------
    def set_initial_pose_in_rviz(self):
        self.get_logger().info("Waiting for the robot's initial pose...")
        self._got_initial_pose = False
        while rclpy.ok() and not self._got_initial_pose:
            self._spin_once(0.1)
        self.get_logger().info("Set initial pose finished.")
        return True

    def get_clicked_point_in_rviz(self):
        self.get_logger().info("Waiting for the robot's clicked point...")
        self._got_clicked_point = False
        while rclpy.ok() and not self._got_clicked_point:
            self._spin_once(0.1)
        p = self.last_clicked_point.point
        self.get_logger().info(
            "Get clicked point (%.2f, %.2f, %.2f)" % (p.x, p.y, p.z)
        )
        return self.last_clicked_point

    def set_goal_in_rviz(self):
        self.get_logger().info("Waiting for the robot's goal...")
        self._got_goal_pose = False
        while rclpy.ok() and not self._got_goal_pose:
            self._spin_once(0.1)
        self.get_logger().info("Set goal finished.")
        time.sleep(1.0)
        return True

    # -------------------- Navigation --------------------
    def move_to(self, x, y, theta, timeout=300.0):
        """Send a single goal and block until it finishes (or timeout)."""
        self.clear_costmaps()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = self.point_to_pose(x, y, theta)

        self.status_code = self.STATUS_PENDING
        self.status_text = "sending"

        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            self.status_code = self.STATUS_ABORTED
            return 0

        self._goal_handle = goal_handle
        self.status_code = self.STATUS_ACTIVE

        result_future = goal_handle.get_result_async()
        self._result_future = result_future

        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if result_future.done():
                break
            if (time.time() - start) > timeout:
                self.get_logger().warn("move_to timeout, cancelling goal.")
                goal_handle.cancel_goal_async()
                self.status_code = self.STATUS_ABORTED
                return 0

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_code = self.STATUS_SUCCEEDED
            self.get_logger().info("Reached point.")
            return 1
        else:
            self.status_code = self.STATUS_ABORTED
            self.get_logger().info("Failed to reach point.")
            return 0

    def _nav_feedback_cb(self, feedback_msg):
        self.status_code = self.STATUS_ACTIVE

    # -------------------- Navigate through poses --------------------
    def nav_through_poses(self, poses, timeout=600.0):
        """
        Navigate through a sequence of poses without stopping at intermediate poses.
        Uses Nav2's NavigateThroughPoses action.

        Args:
            poses: iterable of (x, y, theta) tuples in self.frame_id.
            timeout: total timeout (seconds).

        Returns:
            1 if success, 0 otherwise.
        """
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_through_poses action server not available.")
            return 0

        self.clear_costmaps()

        goal_msg = NavigateThroughPoses.Goal()
        now = self.get_clock().now().to_msg()
        for (x, y, theta) in poses:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose = self.point_to_pose(x, y, theta)
            goal_msg.poses.append(ps)

        self.get_logger().info(f"Navigating through {len(goal_msg.poses)} poses...")
        self.status_code = self.STATUS_PENDING
        self.status_text = "sending poses"

        send_future = self.nav_through_poses_client.send_goal_async(
            goal_msg, feedback_callback=self._ntp_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info("Navigate through poses goal rejected.")
            self.status_code = self.STATUS_ABORTED
            return 0

        self._goal_handle = goal_handle
        self.status_code = self.STATUS_ACTIVE

        result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if result_future.done():
                break
            if (time.time() - start) > timeout:
                self.get_logger().warn("nav_through_poses timeout, cancelling.")
                goal_handle.cancel_goal_async()
                self.status_code = self.STATUS_ABORTED
                return 0

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_code = self.STATUS_SUCCEEDED
            self.get_logger().info("Successfully navigated through all poses.")
            return 1
        else:
            self.status_code = self.STATUS_ABORTED
            self.get_logger().info(f"Navigate through poses finished with status={status}")
            return 0

    def _ntp_feedback_cb(self, feedback_msg):
        """Feedback callback for NavigateThroughPoses action."""
        self.status_code = self.STATUS_ACTIVE
        try:
            fb = feedback_msg.feedback
            self.status_text = f"navigating through poses"
        except AttributeError:
            pass

    # -------------------- Waypoint follower --------------------
    def follow_waypoints(self, waypoints, timeout=600.0):
        """
        Follow a list of waypoints using Nav2's FollowWaypoints action.

        Args:
            waypoints: iterable of (x, y, theta) tuples in self.frame_id.
            timeout:   total timeout (seconds).

        Returns:
            1 if success, 0 otherwise.
        """
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_waypoints action server not available.")
            return 0

        self.clear_costmaps()

        goal_msg = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for (x, y, theta) in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose = self.point_to_pose(x, y, theta)
            goal_msg.poses.append(ps)

        self.get_logger().info(f"Sending {len(goal_msg.poses)} waypoints...")
        self.status_code = self.STATUS_PENDING
        self.status_text = "sending waypoints"

        send_future = self.follow_waypoints_client.send_goal_async(
            goal_msg, feedback_callback=self._wp_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info("Waypoints goal rejected.")
            self.status_code = self.STATUS_ABORTED
            return 0

        self._goal_handle = goal_handle
        self.status_code = self.STATUS_ACTIVE

        result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if result_future.done():
                break
            if (time.time() - start) > timeout:
                self.get_logger().warn("follow_waypoints timeout, cancelling.")
                goal_handle.cancel_goal_async()
                self.status_code = self.STATUS_ABORTED
                return 0

        status = result_future.result().status
        result = result_future.result().result
        missed = list(getattr(result, "missed_waypoints", []))
        if status == GoalStatus.STATUS_SUCCEEDED and not missed:
            self.status_code = self.STATUS_SUCCEEDED
            self.get_logger().info("All waypoints reached.")
            return 1
        else:
            self.status_code = self.STATUS_ABORTED
            self.get_logger().info(
                f"Waypoints finished with status={status}, missed={missed}"
            )
            return 0

    def _wp_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            self.status_text = f"current_waypoint={fb.current_waypoint}"
        except AttributeError:
            pass
        self.status_code = self.STATUS_ACTIVE

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # -------------------- Callbacks --------------------
    def initialpose_callback(self, msg):
        self.initial_pose = msg
        self._got_initial_pose = True

    def amcl_pose_callback(self, msg):
        self.current_pose = msg
        self._got_amcl_pose = True

    def clicked_point_callback(self, msg):
        self.last_clicked_point = msg
        self._got_clicked_point = True

    def goal_callback(self, msg):
        self.last_goal_pose = msg
        self._got_goal_pose = True

    # -------------------- Pose getters --------------------
    def update(self, timeout=0.05):
        """Manually pump callbacks (useful between long sleeps in user code)."""
        self._spin_once(timeout)

    def get_current_pose(self, refresh=True):
        """
        Returns (x, y, yaw) of the latest /amcl_pose.
        If refresh=True, spin once first to grab any pending message.
        """
        if refresh:
            self._spin_once(0.0)
        p = self.current_pose.pose.pose.position
        o = self.current_pose.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return p.x, p.y, yaw

    def get_goal_pose(self):
        p = self.last_goal_pose.pose.position
        o = self.last_goal_pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return p.x, p.y, yaw

    # -------------------- Shutdown --------------------
    def shutdown(self):
        self.get_logger().info("Robot Chassis is shutting down...")
        self.cancel_goal()


# -------------------- Example usage --------------------
def main():
    rclpy.init()
    chassis = RobotChassis2()

    try:
        P = chassis.get_current_pose()
        chassis.get_logger().info("Current pose: %.2f, %.2f, %.2f" % P)

        # Single goal demo
        chassis.move_to(0.5, 0.0, 0.0)
        P = chassis.get_current_pose()
        chassis.get_logger().info("After move: %.2f, %.2f, %.2f" % P)

        # Navigate through poses demo (robot doesn't stop at intermediate poses)
        poses = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
            (0.0, 0.0, -1.57),
        ]
        chassis.nav_through_poses(poses)

        # Waypoint follower demo (robot stops at each waypoint)
        waypoints = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
            (0.0, 0.0, -1.57),
        ]
        chassis.follow_waypoints(waypoints)

    except KeyboardInterrupt:
        pass
    finally:
        chassis.shutdown()
        chassis.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
