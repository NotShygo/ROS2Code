# RoboticArm2.py
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from std_msgs.msg import Header

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    JointConstraint, PositionConstraint, OrientationConstraint,
    PlanningOptions, BoundingVolume, RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from shape_msgs.msg import SolidPrimitive

from tf_transformations import quaternion_from_euler, euler_from_quaternion


class RoboticArm2(Node):
    # status codes (mirrors RobotChassis2)
    STATUS_PENDING   = 0
    STATUS_ACTIVE    = 1
    STATUS_SUCCEEDED = 3
    STATUS_ABORTED   = 4

    def __init__(self,
                 group_name="arm",
                 base_frame="base_link",
                 end_effector_link="tool0",
                 joint_names=None,
                 node_name="robotic_arm",
                 planner_id="RRTConnectkConfigDefault",
                 vel_scaling=0.5,
                 acc_scaling=0.5,
                 joint_state_topic="/joint_states",
                 wait_for_joint_state_timeout=10.0):
        super().__init__(node_name)

        # config
        self.group_name = group_name
        self.base_frame = base_frame
        self.end_effector_link = end_effector_link
        self.joint_names = joint_names  # may be None; auto-filled from /joint_states
        self.planner_id = planner_id
        self.vel_scaling = float(vel_scaling)
        self.acc_scaling = float(acc_scaling)

        # state
        self.current_joint_state = JointState()
        self._got_joint_state = False
        self.status_code = 0
        self.status_text = ""
        self._goal_handle = None

        # action client to move_action
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for /move_action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to /move_action.")

        # service clients (optional, used by FK/IK helpers)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # joint state subscription
        self.create_subscription(
            JointState, joint_state_topic, self._joint_state_cb, 10
        )

        self._wait_for_joint_state(timeout=wait_for_joint_state_timeout)
        self.get_logger().info("RoboticArm2 OK")

    # -------------------- helpers --------------------
    def _spin_once(self, timeout=0.1):
        rclpy.spin_once(self, timeout_sec=timeout)

    def _wait_for_joint_state(self, timeout=10.0):
        self.get_logger().info("Waiting for first /joint_states...")
        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if self._got_joint_state:
                self.get_logger().info(
                    f"Got joint state with {len(self.current_joint_state.name)} joints."
                )
                if self.joint_names is None:
                    self.joint_names = list(self.current_joint_state.name)
                return True
            if (time.time() - start) > timeout:
                self.get_logger().warn("Timed out waiting for /joint_states.")
                return False
        return False

    def _joint_state_cb(self, msg):
        self.current_joint_state = msg
        self._got_joint_state = True

    def update(self, timeout=0.05):
        self._spin_once(timeout)

    # -------------------- core: build & send MoveGroup goal --------------------
    def _build_request(self, constraints):
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.planner_id = self.planner_id
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel_scaling
        req.max_acceleration_scaling_factor = self.acc_scaling
        req.goal_constraints.append(constraints)

        # start = current state (let MoveIt fill it in)
        req.start_state = RobotState()
        req.start_state.is_diff = True
        return req

    def _send_goal(self, motion_request, plan_only=False, timeout=60.0):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_request
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = plan_only
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.status_code = self.STATUS_PENDING
        self.status_text = "sending"
        send_future = self.move_group_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info("MoveGroup goal rejected.")
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
                self.get_logger().warn("MoveGroup timeout, cancelling.")
                goal_handle.cancel_goal_async()
                self.status_code = self.STATUS_ABORTED
                return 0

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_code = self.STATUS_SUCCEEDED
            self.get_logger().info("Motion succeeded.")
            return 1
        else:
            self.status_code = self.STATUS_ABORTED
            self.get_logger().info(f"Motion failed (status={status}).")
            return 0

    def _feedback_cb(self, feedback_msg):
        self.status_code = self.STATUS_ACTIVE
        try:
            self.status_text = feedback_msg.feedback.state
        except AttributeError:
            pass

    # -------------------- joint goal --------------------
    def move_to_joint(self, joint_positions, joint_names=None,
                      tolerance=0.01, timeout=60.0):
        """
        joint_positions: list[float]
        joint_names:     list[str] (optional, defaults to self.joint_names)
        """
        names = joint_names if joint_names is not None else self.joint_names
        if names is None or len(names) != len(joint_positions):
            self.get_logger().error("joint_names/positions size mismatch.")
            return 0

        c = Constraints()
        for n, p in zip(names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(p)
            jc.tolerance_above = float(tolerance)
            jc.tolerance_below = float(tolerance)
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req = self._build_request(c)
        return self._send_goal(req, timeout=timeout)

    # -------------------- pose goal --------------------
    def move_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0,
                     frame_id=None, end_effector_link=None,
                     pos_tol=0.005, ori_tol=0.01, timeout=60.0):
        frame = frame_id or self.base_frame
        link  = end_effector_link or self.end_effector_link
        q = quaternion_from_euler(roll, pitch, yaw)

        # position constraint (small box)
        pos_c = PositionConstraint()
        pos_c.header.frame_id = frame
        pos_c.link_name = link
        pos_c.weight = 1.0
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(pos_tol)]
        bv = BoundingVolume()
        bv.primitives.append(prim)
        center = Pose()
        center.position = Point(x=float(x), y=float(y), z=float(z))
        center.orientation.w = 1.0
        bv.primitive_poses.append(center)
        pos_c.constraint_region = bv

        # orientation constraint
        ori_c = OrientationConstraint()
        ori_c.header.frame_id = frame
        ori_c.link_name = link
        ori_c.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        ori_c.absolute_x_axis_tolerance = float(ori_tol)
        ori_c.absolute_y_axis_tolerance = float(ori_tol)
        ori_c.absolute_z_axis_tolerance = float(ori_tol)
        ori_c.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pos_c)
        c.orientation_constraints.append(ori_c)

        req = self._build_request(c)
        return self._send_goal(req, timeout=timeout)

    def move_to_pose_stamped(self, pose_stamped, **kwargs):
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return self.move_to_pose(p.x, p.y, p.z, r, pi, y,
                                 frame_id=pose_stamped.header.frame_id, **kwargs)

    # -------------------- named target (via IK-less joint goal) --------------------
    # NOTE: ROS2 MoveIt does not expose SRDF named targets through a service.
    # Provide your own dictionary or use joint goals.
    def move_to_named(self, name, named_targets, timeout=60.0):
        """
        named_targets: dict[str, list[float]]
        Example:
            chassis_arm.move_to_named("home", {"home":[0,0,0,0,0,0]})
        """
        if name not in named_targets:
            self.get_logger().error(f"Named target '{name}' not found.")
            return 0
        return self.move_to_joint(named_targets[name], timeout=timeout)

    # -------------------- getters --------------------
    def get_current_joint_positions(self, refresh=True):
        if refresh:
            self._spin_once(0.0)
        return dict(zip(self.current_joint_state.name,
                        self.current_joint_state.position))

    def get_current_pose(self, link=None, timeout=2.0):
        """Compute FK of current joint state for end_effector_link."""
        link = link or self.end_effector_link
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn("FK service not available.")
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [link]
        req.robot_state.joint_state = self.current_joint_state
        req.robot_state.is_diff = False
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        res = future.result()
        if res is None or len(res.pose_stamped) == 0:
            return None
        ps = res.pose_stamped[0]
        p, o = ps.pose.position, ps.pose.orientation
        r, pi, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return p.x, p.y, p.z, r, pi, y

    # -------------------- control --------------------
    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def shutdown(self):
        self.get_logger().info("Robotic Arm is shutting down...")
        self.cancel_goal()


# -------------------- example usage --------------------
def main():
    rclpy.init()
    arm = RoboticArm2(
        group_name="arm",
        base_frame="base_link",
        end_effector_link="tool0",
    )

    try:
        # Current state
        joints = arm.get_current_joint_positions()
        arm.get_logger().info(f"Current joints: {joints}")

        pose = arm.get_current_pose()
        if pose is not None:
            arm.get_logger().info(
                "Current pose: x=%.3f y=%.3f z=%.3f r=%.2f p=%.2f y=%.2f" % pose
            )

        # 1) Joint goal
        arm.move_to_joint([0.0, -1.0, 1.0, 0.0, 1.0, 0.0])

        # 2) Cartesian pose goal
        arm.move_to_pose(0.4, 0.0, 0.3, roll=0.0, pitch=math.pi, yaw=0.0)

        # 3) Named target (you supply the dictionary)
        named = {
            "home":  [0, 0, 0, 0, 0, 0],
            "ready": [0, -0.5, 0.5, 0, 1.0, 0],
        }
        arm.move_to_named("home", named)

    except KeyboardInterrupt:
        pass
    finally:
        arm.shutdown()
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()