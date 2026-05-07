# Gripper2.py
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PlanningOptions, RobotState,
)


class Gripper2(Node):
    """
    Unified gripper helper. Choose backend = "moveit" or "gripper_command".

    MoveIt backend:
        Gripper2(backend="moveit",
                 group_name="gripper",
                 joint_names=["finger_joint"],
                 open_position=[0.04],
                 closed_position=[0.0])

    GripperCommand backend (ros2_control):
        Gripper2(backend="gripper_command",
                 action_name="/gripper_controller/gripper_cmd",
                 open_position=0.04,
                 closed_position=0.0,
                 max_effort=50.0)
    """

    STATUS_PENDING   = 0
    STATUS_ACTIVE    = 1
    STATUS_SUCCEEDED = 3
    STATUS_ABORTED   = 4

    def __init__(self,
                 backend="moveit",
                 # --- MoveIt backend args ---
                 group_name="gripper",
                 joint_names=None,
                 open_position=None,
                 closed_position=None,
                 planner_id="RRTConnectkConfigDefault",
                 vel_scaling=0.5,
                 acc_scaling=0.5,
                 # --- GripperCommand backend args ---
                 action_name="/gripper_controller/gripper_cmd",
                 max_effort=50.0,
                 # --- common ---
                 joint_state_topic="/joint_states",
                 node_name="gripper"):
        super().__init__(node_name)

        self.backend = backend
        self.group_name = group_name
        self.joint_names = joint_names
        self.open_position = open_position
        self.closed_position = closed_position
        self.planner_id = planner_id
        self.vel_scaling = float(vel_scaling)
        self.acc_scaling = float(acc_scaling)
        self.max_effort = float(max_effort)

        self.status_code = 0
        self.status_text = ""
        self._goal_handle = None

        self.current_joint_state = JointState()
        self._got_joint_state = False
        self.create_subscription(JointState, joint_state_topic,
                                 self._joint_state_cb, 10)

        if backend == "moveit":
            self._client = ActionClient(self, MoveGroup, "/move_action")
            self.get_logger().info("Waiting for /move_action server (gripper)...")
            self._client.wait_for_server()
            self.get_logger().info("Gripper2 (moveit) connected.")
        elif backend == "gripper_command":
            self._client = ActionClient(self, GripperCommand, action_name)
            self.get_logger().info(f"Waiting for {action_name} server...")
            self._client.wait_for_server()
            self.get_logger().info(f"Gripper2 (gripper_command) connected on {action_name}.")
        else:
            raise ValueError(f"Unknown backend: {backend}")

        self.get_logger().info("Gripper2 OK")

    # -------------------- helpers --------------------
    def _spin_once(self, t=0.1): rclpy.spin_once(self, timeout_sec=t)

    def _joint_state_cb(self, msg):
        self.current_joint_state = msg
        self._got_joint_state = True

    def update(self, t=0.05): self._spin_once(t)

    def _feedback_cb(self, _):
        self.status_code = self.STATUS_ACTIVE

    # -------------------- MoveIt backend --------------------
    def _send_moveit_joint_goal(self, positions, tolerance=0.005, timeout=20.0):
        if self.joint_names is None or len(self.joint_names) != len(positions):
            self.get_logger().error("Gripper joint_names/positions size mismatch.")
            return 0

        c = Constraints()
        for n, p in zip(self.joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(p)
            jc.tolerance_above = float(tolerance)
            jc.tolerance_below = float(tolerance)
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.planner_id = self.planner_id
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = self.vel_scaling
        req.max_acceleration_scaling_factor = self.acc_scaling
        req.goal_constraints.append(c)
        req.start_state = RobotState()
        req.start_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        return self._run_action(goal, timeout)

    # -------------------- GripperCommand backend --------------------
    def _send_gripper_command(self, position, max_effort=None, timeout=10.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort if max_effort is not None
                                        else self.max_effort)
        return self._run_action(goal, timeout)

    # -------------------- common runner --------------------
    def _run_action(self, goal_msg, timeout):
        self.status_code = self.STATUS_PENDING
        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().info("Gripper goal rejected.")
            self.status_code = self.STATUS_ABORTED
            return 0

        self._goal_handle = gh
        self.status_code = self.STATUS_ACTIVE
        result_future = gh.get_result_async()
        start = time.time()
        while rclpy.ok():
            self._spin_once(0.1)
            if result_future.done():
                break
            if (time.time() - start) > timeout:
                self.get_logger().warn("Gripper timeout, cancelling.")
                gh.cancel_goal_async()
                self.status_code = self.STATUS_ABORTED
                return 0

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status_code = self.STATUS_SUCCEEDED
            return 1
        else:
            self.status_code = self.STATUS_ABORTED
            self.get_logger().info(f"Gripper finished with status={status}")
            # GripperCommand often "fails" when stalled on an object — that's
            # actually how you know you grasped something. Treat stalled as ok.
            try:
                res = result_future.result().result
                if hasattr(res, "stalled") and res.stalled:
                    self.get_logger().info("Gripper stalled (object grasped).")
                    self.status_code = self.STATUS_SUCCEEDED
                    return 1
            except Exception:
                pass
            return 0

    # -------------------- public API --------------------
    def open(self, timeout=10.0):
        self.get_logger().info("Opening gripper...")
        if self.backend == "moveit":
            return self._send_moveit_joint_goal(self.open_position, timeout=timeout)
        else:
            return self._send_gripper_command(self.open_position, timeout=timeout)

    def close(self, timeout=10.0):
        self.get_logger().info("Closing gripper...")
        if self.backend == "moveit":
            return self._send_moveit_joint_goal(self.closed_position, timeout=timeout)
        else:
            return self._send_gripper_command(self.closed_position, timeout=timeout)

    def move_to(self, position, timeout=10.0):
        """Send a custom position (float for gripper_command, list for moveit)."""
        if self.backend == "moveit":
            return self._send_moveit_joint_goal(position, timeout=timeout)
        else:
            return self._send_gripper_command(position, timeout=timeout)

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def shutdown(self):
        self.get_logger().info("Gripper shutting down...")
        self.cancel_goal()


# -------------------- example --------------------
def main():
    rclpy.init()

    # --- Option A: MoveIt-controlled gripper ---
    gripper = Gripper2(
        backend="moveit",
        group_name="gripper",
        joint_names=["finger_joint"],
        open_position=[0.04],
        closed_position=[0.0],
    )

    # --- Option B: ros2_control GripperCommand ---
    # gripper = Gripper2(
    #     backend="gripper_command",
    #     action_name="/gripper_controller/gripper_cmd",
    #     open_position=0.04,
    #     closed_position=0.0,
    #     max_effort=50.0,
    # )

    try:
        gripper.open()
        time.sleep(1.0)
        gripper.close()
    except KeyboardInterrupt:
        pass
    finally:
        gripper.shutdown()
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
