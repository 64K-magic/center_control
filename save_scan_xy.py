#!/usr/bin/env python3
"""
Subscribe one frame of /scan and save XY projection to PNG.
Usage:
    python3 save_scan_xy.py [output_png]
Default:
    output_png = /home/jfby/jfby_ws/scan_xy.png
"""
import sys
import rclpy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else '/home/jfby/jfby_ws/scan_xy.png'

    rclpy.init()
    node = rclpy.create_node('save_scan_xy')
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    node.create_subscription(LaserScan, '/scan', cb, rclpy.qos.qos_profile_sensor_data)

    print('Waiting for /scan...')
    for i in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if msg:
            break

    if not msg:
        print('ERROR: No /scan received')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ranges = np.array(msg.ranges)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
    r = ranges[valid]
    a = angles[valid]
    x = r * np.cos(a)
    y = r * np.sin(a)

    print(f'Received scan: total={len(ranges)}, valid={len(r)}')
    print(f'range_min={msg.range_min}, range_max={msg.range_max}')
    print(f'x range: [{x.min():.2f}, {x.max():.2f}]')
    print(f'y range: [{y.min():.2f}, {y.max():.2f}]')

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.scatter(x, y, s=1, c='blue', alpha=0.5)
    ax.scatter(0, 0, s=100, c='red', marker='o', label='robot center')
    ax.set_xlabel('x (m) - forward')
    ax.set_ylabel('y (m) - left')
    ax.set_title('/scan XY projection (odin1_base_link frame)')
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_xlim(-5, 10)
    ax.set_ylim(-5, 5)
    fig.savefig(out_path, dpi=150)
    plt.close()

    print(f'Saved to {out_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
