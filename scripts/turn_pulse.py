#!/usr/bin/env python3
"""Timed angular cmd_vel pulse. Usage: turn_pulse.py [rad_per_s] [seconds]"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

def main():
    rclpy.init()
    node = Node('turn_pulse')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10:
            print('ERROR: no /cmd_vel subscriber'); sys.exit(1)
    print(f'Subscriber found after {time.time()-t0:.2f}s')
    m = Twist(); m.angular.z = RATE
    print(f'TURNING: angular.z={RATE} for {DUR}s  (expect {RATE*DUR:.3f} rad = {RATE*DUR*57.2958:.1f} deg)')
    t0 = time.time(); n = 0
    while time.time() - t0 < DUR:
        pub.publish(m); n += 1; time.sleep(0.05)
    for _ in range(10):
        pub.publish(Twist()); time.sleep(0.02)
    print(f'STOP. {n} msgs over {time.time()-t0:.2f}s')
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
