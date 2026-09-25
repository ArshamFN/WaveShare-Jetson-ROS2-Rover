#!/usr/bin/env python3
"""
drive_pulse.py — deterministic timed cmd_vel pulse.

`ros2 topic pub` spends its first ~1 s on DDS discovery, so a short
`timeout` window can expire before any command is delivered. This waits
for a confirmed subscriber, then publishes for an exact duration, then
sends an explicit zero.

Usage:  python3 ~/drive_pulse.py [speed] [seconds]
        python3 ~/drive_pulse.py 1.0 1.5
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
RATE_HZ = 20.0


def main():
    rclpy.init()
    node = Node('drive_pulse')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print(f'Waiting for a /cmd_vel subscriber...')
    t_wait = time.time()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t_wait > 10.0:
            print('ERROR: no subscriber on /cmd_vel after 10 s. '
                  'Is rover_driver_node running?')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    print(f'Subscriber found after {time.time() - t_wait:.2f} s.')

    msg = Twist()
    msg.linear.x = SPEED

    print(f'DRIVING: linear.x={SPEED} for {DURATION:.2f} s')
    t0 = time.time()
    n = 0
    while time.time() - t0 < DURATION:
        pub.publish(msg)
        n += 1
        time.sleep(1.0 / RATE_HZ)
    elapsed = time.time() - t0

    stop = Twist()
    for _ in range(10):
        pub.publish(stop)
        time.sleep(0.02)

    print(f'STOP sent. Published {n} msgs over {elapsed:.2f} s.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
