#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class RoverDriverNode(Node):
    def __init__(self):
        super().__init__('rover_driver')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.ser = serial.Serial('/dev/rover', 115200, timeout=1)
        self.get_logger().info('Rover driver node started, listening on /cmd_vel')

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        # Convert to left/right wheel speeds using unicycle drive kinematics
        left = linear - angular * 0.5
        right = linear + angular * 0.5
        # Clamp to [-1, 1]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        command = json.dumps({'T': 1, 'L': round(left, 3), 'R': round(right, 3)}) + '\n'
        self.ser.write(command.encode())
        self.get_logger().info(f'CMD: linear={linear:.2f} angular={angular:.2f} -> L={left:.3f} R={right:.3f}')

    def destroy_node(self):
        self.ser.write(b'{"T":1,"L":0,"R":0}\n')
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoverDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
