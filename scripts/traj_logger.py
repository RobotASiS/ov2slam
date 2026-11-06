#!/usr/bin/env python3
"""
Logger przejazdu: zapisuje wszystko co trzeba, zeby potem splotowac co robil robot
wedlug ROSa (SLAM, EKF, MPC, plan, komendy) i co bylo na mapie w GUI.

Kazde uruchomienie = nowy katalog <log_dir>/run_YYYYmmdd_HHMMSS/ z plikami:
  vo_pose.csv    - poza ze SLAMa po konwersji osi (/vo_pose_cov), punkt kamery
  odom.csv       - poza z EKF (/odometry/filtered) - to samo co kropka w GUI
  mpc.csv        - stan MPC (/mpc/debug): punkt tylnej osi, referencja, komendy, czas solvera
  cmd_vel.csv    - komendy wyslane do mostka
  encoders.csv   - enkodery z ESP32
  goals.csv      - klikniete cele i anulowania
  plans.csv      - kazda trasa z plannera (plan_id, punkty)
  maps/map_XXXX.npz - snapshoty siatki /map (co map_period s)

Plot: scripts/plot_run.py <katalog_runa>
"""
import csv
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Empty, Float64MultiArray, Int64MultiArray


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


MPC_FIELDS = ['x_P', 'y_P', 'yaw', 'ref_x', 'ref_y', 'ref_yaw', 'v_target',
              'v_cmd', 'psi_cmd', 'goal_dist', 'rem_dist', 'solve_ms']


class TrajLogger(Node):
    def __init__(self):
        super().__init__('traj_logger')
        self.declare_parameter('log_dir', os.path.join(os.getcwd(), 'robot_logs'))
        self.declare_parameter('map_period', 2.0)

        base = os.path.expanduser(self.get_parameter('log_dir').value)
        self.run_dir = os.path.join(base, datetime.now().strftime('run_%Y%m%d_%H%M%S'))
        os.makedirs(os.path.join(self.run_dir, 'maps'), exist_ok=True)
        self.map_period = float(self.get_parameter('map_period').value)

        self.files = []
        self.w_vo = self.open_csv('vo_pose.csv', ['t', 't_msg', 'x', 'y', 'z', 'yaw'])
        self.w_odom = self.open_csv('odom.csv', ['t', 't_msg', 'x', 'y', 'z', 'yaw', 'vx', 'wz'])
        self.w_mpc = self.open_csv('mpc.csv', ['t'] + MPC_FIELDS)
        self.w_cmd = self.open_csv('cmd_vel.csv', ['t', 'v', 'angular_z'])
        self.w_enc = self.open_csv('encoders.csv', ['t', 'left', 'right'])
        self.w_goal = self.open_csv('goals.csv', ['t', 'event', 'x', 'y', 'yaw'])
        self.w_plan = self.open_csv('plans.csv', ['t', 'plan_id', 'idx', 'x', 'y'])

        self.plan_id = 0
        self.map_id = 0
        self.last_map_save = 0.0

        self.create_subscription(Odometry, '/vo_pose_cov', self.vo_cb, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 50)
        self.create_subscription(Float64MultiArray, '/mpc/debug', self.mpc_cb, 50)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 50)
        self.create_subscription(Int64MultiArray, '/esp32/encoders', self.enc_cb, 50)
        self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)
        self.create_subscription(Empty, '/goal_cancel', self.cancel_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 2)

        # flush co sekunde, zeby po ubiciu launcha nic nie przepadlo
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Loguje przejazd do {self.run_dir}')

    def open_csv(self, name, header):
        f = open(os.path.join(self.run_dir, name), 'w', newline='')
        w = csv.writer(f)
        w.writerow(header)
        self.files.append(f)
        return w

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def vo_cb(self, msg):
        p = msg.pose.pose
        self.w_vo.writerow([self.now(), stamp_to_sec(msg.header.stamp),
                            p.position.x, p.position.y, p.position.z, yaw_from_q(p.orientation)])

    def odom_cb(self, msg):
        p = msg.pose.pose
        tw = msg.twist.twist
        self.w_odom.writerow([self.now(), stamp_to_sec(msg.header.stamp),
                              p.position.x, p.position.y, p.position.z, yaw_from_q(p.orientation),
                              tw.linear.x, tw.angular.z])

    def mpc_cb(self, msg):
        self.w_mpc.writerow([self.now()] + list(msg.data))

    def cmd_cb(self, msg):
        self.w_cmd.writerow([self.now(), msg.linear.x, msg.angular.z])

    def enc_cb(self, msg):
        d = list(msg.data) + [0, 0]
        self.w_enc.writerow([self.now(), d[0], d[1]])

    def goal_cb(self, msg):
        p = msg.pose
        self.w_goal.writerow([self.now(), 'goal', p.position.x, p.position.y, yaw_from_q(p.orientation)])

    def cancel_cb(self, _msg):
        self.w_goal.writerow([self.now(), 'cancel', '', '', ''])

    def plan_cb(self, msg):
        t = self.now()
        for i, ps in enumerate(msg.poses):
            self.w_plan.writerow([t, self.plan_id, i, ps.pose.position.x, ps.pose.position.y])
        self.plan_id += 1

    def map_cb(self, msg):
        t = self.now()
        if t - self.last_map_save < self.map_period:
            return
        self.last_map_save = t
        info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        np.savez_compressed(
            os.path.join(self.run_dir, 'maps', f'map_{self.map_id:04d}.npz'),
            t=t, grid=grid, resolution=info.resolution,
            origin_x=info.origin.position.x, origin_y=info.origin.position.y)
        self.map_id += 1

    def flush(self):
        for f in self.files:
            f.flush()

    def close(self):
        for f in self.files:
            f.close()


def main(args=None):
    rclpy.init(args=args)
    node = TrajLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
