import rclpy
from RobotChassis2 import RobotChassis2
import time

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('demo')

    chassis = RobotChassis2()
    waypoints = [(-2.0867932087477667, 0.012138476225076413, 0.2942108767012347),
                (-0.5902562360389836, 0.6370634666823878, -2.10064481317963),
                (1.1583672506373408, 0.5948705879373494, -0.2091009385809236)]
    try: 
        chassis.nav_through_poses(waypoints)
    except KeyboardInterrupt:
        pass
    finally:
        chassis.shutdown()
        chassis.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
