#!/usr/bin/env python3

import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

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
        self.declare_parameter('goal_timeout_sec', 15.0)
        self.declare_parameter('publish_steering_angle', True)

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
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').value
        self.publish_steering_angle = self.get_parameter('publish_steering_angle').value

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
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
        self.goal_start_time = None
        self.step_count = 0

        self.init_solver()
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f' Horizon N={self.N}, dt={self.dt}s, v_max={self.v_max}m/s, Failsafe={self.goal_timeout_sec}s, Ackermann_steer={self.publish_steering_angle}'
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

    def path_callback(self, msg: Path):
        if len(msg.poses) >= 2:
            self.current_path = msg.poses
            self.goal_start_time = self.get_clock().now()
            self.get_logger().info(
                f'MPC received new reference path with {len(msg.poses)} waypoints. Failsafe timeout: {self.goal_timeout_sec}s.'
            )
        else:
            self.current_path = None
            self.goal_start_time = None
            self.pub_cmd_vel.publish(Twist())

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

        if rem_dist <= self.goal_tol:
            return None, 0.0, True

        v_target = self.v_max * np.tanh(rem_dist / self.d_slowdown)
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

    def control_loop(self):
        if self.current_pose is None or self.current_path is None:
            return

        # Failsafe: zatrzymanie jeśli cel nie zostanie osiągnięty w zadanym limicie czasu
        if self.goal_start_time is not None and self.goal_timeout_sec > 0.0:
            elapsed_sec = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed_sec > self.goal_timeout_sec:
                self.get_logger().warn(
                    f'FAILSAFE ZADZIAŁAŁ: Robot nie osiągnął celu w ciągu {self.goal_timeout_sec:.1f} s! Zatrzymanie napędu.'
                )
                self.pub_cmd_vel.publish(Twist())
                self.current_path = None
                self.goal_start_time = None
                return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = get_yaw_from_quaternion(self.current_pose.orientation)

        ref_traj, v_target, goal_reached = self.sample_reference_trajectory(
            robot_x, robot_y, robot_yaw
        )

        cmd = Twist()
        if goal_reached:
            self.pub_cmd_vel.publish(cmd)
            if self.current_path is not None:
                self.get_logger().info('Goal reached! Robot stopped.')
                self.current_path = None
                self.goal_start_time = None
            return

        p_val = np.hstack([[robot_x, robot_y, robot_yaw], ref_traj.flatten()])

        sol = self.solver(x0=self.u0, p=p_val, lbx=self.lbx, ubx=self.ubx)
        u_opt = sol['x'].full().reshape((self.N, 2))

        self.u0 = np.vstack([u_opt[1:], u_opt[-1:]]).reshape(-1, 1)

        v_cmd = float(u_opt[0, 0])
        psi_cmd = float(u_opt[0, 1])
        omega_cmd = (v_cmd / self.L) * np.tan(psi_cmd)

        cmd.linear.x = v_cmd
        # Dla podwozia Ackermanna i mostka esp32_i2c_bridge_node wysyłamy kąt skrętu [rad]
        cmd.angular.z = psi_cmd if self.publish_steering_angle else omega_cmd

        self.pub_cmd_vel.publish(cmd)

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
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
