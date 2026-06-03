import math, time, rclpy
from RoboticArm2 import RoboticArm2
from Gripper2 import Gripper2

if __name__ == "__main__":
    rclpy.init()
    # Check param by 
    #ros2 node list
    # ros2 param get /move_group robot_description_semantic (replace /move_group by the node name)
    arm = RoboticArm2(
        group_name="panda_arm",
        base_frame="panda_link0",
        end_effector_link="panda_link8",
        joint_names=[
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6","panda_joint7",
        ],
    )

    gripper = Gripper2(
        backend="moveit",
        group_name="hand",
        joint_names=["panda_finger_joint1", "panda_finger_joint2"],
        open_position=[0.04, 0.04],
        closed_position=[0.0, 0.0],
    )

    try:
        # Joint goal (7 values for Panda)
        arm.move_to_joint([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

        pose = arm.get_current_pose()
        print(pose)

        # Pose goal
        arm.move_to_pose(0.4, 0.0, 0.4, roll=math.pi, pitch=0.0, yaw=0.0)
        
        gripper.open();  time.sleep(0.5)
        gripper.close(); time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        gripper.shutdown(); arm.shutdown()
        gripper.destroy_node(); arm.destroy_node()
        rclpy.shutdown()
