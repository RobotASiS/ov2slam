#!/usr/bin/env python3

import time
import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, Float64MultiArray, Int64MultiArray
from collections import deque

def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

def normalize_angle_diff(target, source):
    diff = (target - source + np.pi) % (2.0 * np.pi) - np.pi
    return source + diff

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        self.declare_parameter('horizon_N', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('d_slowdown', 0.8)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('mode', 0)
        self.declare_parameter('q_x', 5.0)
        self.declare_parameter('q_y', 5.0)
        self.declare_parameter('q_theta', 1.0)
        self.declare_parameter('r_v', 0.5)
        self.declare_parameter('r_psi', 0.5)
        self.declare_parameter('psi_max_deg', 35.0)
        self.declare_parameter('publish_steering_angle', True)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('max_pose_speed', 1.0)
        self.declare_parameter('pose_check_window', 0.5)
        # Kontrola SLAM vs enkodery: kola przejechaly duzo, a SLAM prawie nic -> SLAM nie nadaza
        self.declare_parameter('slam_check_enabled', True)
        self.declare_parameter('slam_check_window', 1.0)      # [s]
        self.declare_parameter('slam_check_min_enc_dist', 0.15)  # [m] ponizej nie sprawdzamy
        self.declare_parameter('slam_check_min_ratio', 0.3)   # SLAM / enkodery
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('ticks_per_rev', 1000.0)

        self.N = self.get_parameter('horizon_N').value
        self.dt = self.get_parameter('dt').value
        self.L = self.get_parameter('wheelbase').value
        self.v_max = self.get_parameter('v_max').value
        self.d_slowdown = self.get_parameter('d_slowdown').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.mode = self.get_parameter('mode').value
        self.q_x = self.get_parameter('q_x').value
        self.q_y = self.get_parameter('q_y').value
        self.q_theta = self.get_parameter('q_theta').value
        self.r_v = self.get_parameter('r_v').value
        self.r_psi = self.get_parameter('r_psi').value
        self.psi_max_deg = self.get_parameter('psi_max_deg').value
        self.publish_steering_angle = self.get_parameter('publish_steering_angle').value
        self.cam_offset_x = self.get_parameter('cam_offset_x').value
        self.max_pose_speed = self.get_parameter('max_pose_speed').value
        self.pose_check_window = self.get_parameter('pose_check_window').value
        self.slam_check_enabled = self.get_parameter('slam_check_enabled').value
        self.slam_check_window = self.get_parameter('slam_check_window').value
        self.slam_check_min_enc_dist = self.get_parameter('slam_check_min_enc_dist').value
        self.slam_check_min_ratio = self.get_parameter('slam_check_min_ratio').value
        self.m_per_tick = 2.0 * np.pi * self.get_parameter('wheel_radius').value / \
            self.get_parameter('ticks_per_rev').value
        self.enc_dist = None          # skumulowana droga srodka osi z enkoderow [m]
        self.slam_check_hist = deque()

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.sub_enc = self.create_subscription(
            Int64MultiArray,
            '/esp32/encoders',
            self.encoders_callback,
            10
        )
        self.sub_path = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.current_path = None
        self.step_count = 0
        self.prev_pose_xy = None
        self.prev_pose_time = None
        self.pub_goal_cancel = self.create_publisher(Empty, '/goal_cancel', 10)
        # Wewnetrzny stan MPC do logowania (traj_logger.py), kolejnosc pol w publish_debug()
        self.pub_debug = self.create_publisher(Float64MultiArray, '/mpc/debug', 10)
        self.last_goal_dist = float('nan')
        self.last_rem_dist = float('nan')

        self.init_solver()
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f' Horizon N={self.N}, dt={self.dt}s, v_max={self.v_max}m/s, Ackermann_steer={self.publish_steering_angle}'
        )

    def init_solver(self):
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = 3

        v = ca.SX.sym('v')
        psi = ca.SX.sym('psi')
        controls = ca.vertcat(v, psi)
        n_controls = 2

        rhs = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            v * (1.0 / self.L) * ca.tan(psi)
        )
        f = ca.Function('f', [states, controls], [rhs])

        U = ca.SX.sym('U', n_controls, self.N)
        P = ca.SX.sym('P', n_states + self.N * n_states)

        X = [P[0:3]]
        for k in range(self.N):
            st = X[k]
            con = U[:, k]
            st_next = st + self.dt * f(st, con)
            X.append(st_next)

        X = ca.horzcat(*X)

        Q = np.diag([self.q_x, self.q_y, self.q_theta])
        R = np.diag([self.r_v, self.r_psi])

        obj = 0
        for k in range(self.N):
            st = X[:, k + 1]
            con = U[:, k]
            ref = P[3 + k * 3 : 3 + (k + 1) * 3]
            err = st - ref
            obj += ca.mtimes([err.T, Q, err]) + ca.mtimes([con.T, R, con])

        OPT_variables = ca.reshape(U, n_controls * self.N, 1)

        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'p': P
        }

        opts = {
            'ipopt.max_iter': 100,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        psi_max_rad = np.radians(self.psi_max_deg)
        lbx = []
        ubx = []
        for _ in range(self.N):
            lbx.extend([-self.v_max, -psi_max_rad])
            ubx.extend([self.v_max, psi_max_rad])

        self.lbx = np.array(lbx)
        self.ubx = np.array(ubx)
        self.u0 = np.zeros((n_controls * self.N, 1))

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def encoders_callback(self, msg: Int64MultiArray):
        if len(msg.data) < 2:
            return
        # srodek osi = srednia kol; mostek juz odwraca znak lewego enkodera
        self.enc_dist = 0.5 * (msg.data[0] + msg.data[1]) * self.m_per_tick

    def slam_lagging(self, x, y):
        # Porownanie drogi z enkoderow i ze SLAMa w oknie slam_check_window.
        # Przypadek z logow: kola zrobily 1.1 m po luku, SLAM pokazal 6 cm, MPC krecil do oporu.
        if not self.slam_check_enabled or self.enc_dist is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        self.slam_check_hist.append((now, self.enc_dist, x, y))
        while len(self.slam_check_hist) > 1 and now - self.slam_check_hist[1][0] >= self.slam_check_window:
            self.slam_check_hist.popleft()

        t_old, enc_old, x_old, y_old = self.slam_check_hist[0]
        if now - t_old < self.slam_check_window:
            return False

        d_enc = abs(self.enc_dist - enc_old)
        d_slam = float(np.hypot(x - x_old, y - y_old))
        if d_enc < self.slam_check_min_enc_dist:
            return False
        if d_slam < self.slam_check_min_ratio * d_enc:
            self.get_logger().error(
                f'SLAM nie nadaza: kola {d_enc:.2f} m, SLAM {d_slam:.2f} m w {now - t_old:.1f} s - stop'
            )
            return True
        return False

    def path_callback(self, msg: Path):
        if len(msg.poses) >= 2:
            # Planner przelicza trase co chwile - log i reset okna skokow tylko przy nowym celu
            if self.current_path is None:
                self.prev_pose_xy = None
                self.slam_check_hist.clear()
                self.get_logger().info(
                    f'MPC received new reference path with {len(msg.poses)} waypoints.'
                )
            self.current_path = msg.poses
        else:
            self.current_path = None
            self.pub_cmd_vel.publish(Twist())

    def pose_jumped(self, x, y):
        # Robot fizycznie nie moze jechac szybciej niz max_pose_speed, wiec taki ruch
        # pozycji oznacza, ze SLAM stracil tracking. Porownanie w oknie pose_check_window,
        # bo pozycja przychodzi skokami co klatke i krotszy odcinek daje falszywe alarmy
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_pose_xy is None:
            self.prev_pose_xy = (x, y)
            self.prev_pose_time = now
            return False

        elapsed = now - self.prev_pose_time
        if elapsed < self.pose_check_window:
            return False

        dist = np.hypot(x - self.prev_pose_xy[0], y - self.prev_pose_xy[1])
        self.prev_pose_xy = (x, y)
        self.prev_pose_time = now

        jumped = dist / elapsed > self.max_pose_speed
        if jumped:
            self.get_logger().error(
                f'Skok pozycji: {dist:.2f} m w {elapsed:.2f} s - stop, wyslij nowy cel'
            )
        return jumped

    def sample_reference_trajectory(self, current_x, current_y, current_yaw):
        if not self.current_path:
            return None, 0.0, True

        path_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in self.current_path])
        dists = np.linalg.norm(path_pts - np.array([current_x, current_y]), axis=1)
        min_idx = int(np.argmin(dists))

        if min_idx >= len(path_pts) - 1:
            rem_dist = 0.0
        else:
            diffs = np.diff(path_pts[min_idx:], axis=0)
            rem_dist = float(np.sum(np.linalg.norm(diffs, axis=1)))

        goal_dist = float(np.linalg.norm(path_pts[-1] - np.array([current_x, current_y])))
        self.last_goal_dist = goal_dist
        self.last_rem_dist = rem_dist

        # Jedyny warunek konca: robot jest przy celu
        if goal_dist <= self.goal_tol:
            self.get_logger().info('Goal reached! Robot stopped.')
            self.pub_goal_cancel.publish(Empty())
            return None, 0.0, True

        # Hamowanie liczone od wiekszej z odleglosci, zeby robot nie stanal,
        # gdy zjechal z konca sciezki a cel jest jeszcze daleko
        v_target = self.v_max * np.tanh(max(rem_dist, goal_dist) / self.d_slowdown)
        ds = v_target * self.dt

        ref_pts = []
        curr_seg_idx = min_idx
        accum_dist = 0.0

        for k in range(1, self.N + 1):
            target_dist = k * ds
            while accum_dist < target_dist and curr_seg_idx < len(path_pts) - 1:
                p1 = path_pts[curr_seg_idx]
                p2 = path_pts[curr_seg_idx + 1]
                seg_len = np.linalg.norm(p2 - p1)
                if accum_dist + seg_len >= target_dist:
                    alpha = (target_dist - accum_dist) / max(seg_len, 1e-6)
                    curr_pt = p1 + alpha * (p2 - p1)
                    theta_seg = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
                    break
                else:
                    accum_dist += seg_len
                    curr_seg_idx += 1
                    curr_pt = path_pts[curr_seg_idx]

            if curr_seg_idx >= len(path_pts) - 1:
                curr_pt = path_pts[-1]
                if len(path_pts) >= 2:
                    theta_seg = np.arctan2(path_pts[-1][1] - path_pts[-2][1], path_pts[-1][0] - path_pts[-2][0])
                else:
                    theta_seg = current_yaw

            theta_unwrapped = normalize_angle_diff(theta_seg, current_yaw)
            ref_pts.append([curr_pt[0], curr_pt[1], theta_unwrapped])

        return np.array(ref_pts), v_target, False

    def publish_debug(self, x, y, yaw, ref0, v_target, v_cmd, psi_cmd, solve_ms):
        # [x_P, y_P, yaw, ref_x, ref_y, ref_yaw, v_target, v_cmd, psi_cmd, goal_dist, rem_dist, solve_ms]
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(yaw),
                    float(ref0[0]), float(ref0[1]), float(ref0[2]),
                    float(v_target), float(v_cmd), float(psi_cmd),
                    float(self.last_goal_dist), float(self.last_rem_dist), float(solve_ms)]
        self.pub_debug.publish(msg)

    def control_loop(self):
        if self.current_pose is None or self.current_path is None:
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = get_yaw_from_quaternion(self.current_pose.orientation)

        if self.pose_jumped(robot_x, robot_y) or self.slam_lagging(robot_x, robot_y):
            self.current_path = None
            self.pub_goal_cancel.publish(Empty())
            self.u0 = np.zeros_like(self.u0)
            self.pub_cmd_vel.publish(Twist())
            return

        # odometria to poza kamery (punkt K), model liczy wzgledem srodka tylnej osi (punkt P)
        robot_x -= self.cam_offset_x * np.cos(robot_yaw)
        robot_y -= self.cam_offset_x * np.sin(robot_yaw)

        ref_traj, v_target, goal_reached = self.sample_reference_trajectory(
            robot_x, robot_y, robot_yaw
        )

        cmd = Twist()
        if goal_reached:
            self.pub_cmd_vel.publish(cmd)
            self.current_path = None
            return

        p_val = np.hstack([[robot_x, robot_y, robot_yaw], ref_traj.flatten()])

        t_solve = time.perf_counter()
        sol = self.solver(x0=self.u0, p=p_val, lbx=self.lbx, ubx=self.ubx)
        solve_ms = (time.perf_counter() - t_solve) * 1e3
        u_opt = sol['x'].full().reshape((self.N, 2))

        self.u0 = np.vstack([u_opt[1:], u_opt[-1:]]).reshape(-1, 1)

        v_cmd = float(u_opt[0, 0])
        psi_cmd = float(u_opt[0, 1])
        omega_cmd = (v_cmd / self.L) * np.tan(psi_cmd)

        cmd.linear.x = v_cmd
        # Dla podwozia Ackermanna i mostka esp32_i2c_bridge_node wysyłamy kąt skrętu [rad]
        cmd.angular.z = psi_cmd if self.publish_steering_angle else omega_cmd

        self.pub_cmd_vel.publish(cmd)
        self.publish_debug(robot_x, robot_y, robot_yaw, ref_traj[0], v_target, v_cmd, psi_cmd, solve_ms)

        self.step_count += 1
        if self.step_count % 10 == 0:
            self.get_logger().info(
                f'[MPC] tracking: pos=({robot_x:.2f}, {robot_y:.2f}), v_cmd={v_cmd:.2f} m/s, psi={np.degrees(psi_cmd):.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
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
