#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Joint parameters
        self.joint_positions = {
            'base_to_middle': 0.0,
            'middle_to_upper': 0.0,
            'upper_to_effector': 0.0
        }

        self.start_time = self.get_clock().now()

        self.get_logger().info('Robot Controller started!')
        self.get_logger().info('Controlling joints: base_to_middle, middle_to_upper, upper_to_effector')

    def publish_joint_states(self):
        # Calculate time since start
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        # Update joint positions with sinusoidal movement
        self.joint_positions['base_to_middle'] = math.sin(elapsed_time * 0.5) * 1.5
        self.joint_positions['middle_to_upper'] = math.sin(elapsed_time * 0.7 + 1.0) * 1.0
        self.joint_positions['upper_to_effector'] = math.sin(elapsed_time * 0.9 + 2.0) * 0.8

        # Create JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_positions.keys())
        joint_state.position = list(self.joint_positions.values())
        joint_state.velocity = [0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0]

        # Publish joint states
        self.joint_pub.publish(joint_state)

        # Log current positions every 2 seconds
        self.get_logger().info(f'Joint positions: {dict(zip(joint_state.name, joint_state.position))}',
                               throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down robot controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()