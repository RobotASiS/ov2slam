#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import casadi as ca
import math

class CasadiMPCNode(Node):
    def __init__(self):
        super().__init__('casadi_mpc_node')

        # Deklaracja parametrów
        self.declare_parameter('wheelbase', 0.20)      # L [m]
        self.declare_parameter('max_speed', 0.40)      # v_max [m/s]
        self.declare_parameter('min_speed', 0.0)       # v_min [m/s]
        self.declare_parameter('max_steer', 0.60)      # delta_max [rad] (~34 deg)
        self.declare_parameter('horizon_n', 10)        # N
        self.declare_parameter('dt', 0.1)              # dt [s]
        self.declare_parameter('control_rate_hz', 10.0)# f [Hz]
        self.declare_parameter('goal_tolerance', 0.15) # tolerancja dotarcia do celu [m]

        self.L = self.get_parameter('wheelbase').value
        self.v_max = self.get_parameter('max_speed').value
        self.v_min = self.get_parameter('min_speed').value
        self.delta_max = self.get_parameter('max_steer').value
        self.N = self.get_parameter('horizon_n').value
        self.dt = self.get_parameter('dt').value
        self.rate = self.get_parameter('control_rate_hz').value
        self.goal_tol = self.get_parameter('goal_tolerance').value

        # Stan robota
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.odom_received = False

        # Ścieżka referencyjna z A*
        self.path_points = []
        self.last_delta = 0.0

        # Budowa problemu MPC w CasADi
        self.setup_mpc()

        # Subskrypcje i publikacje
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pred_path_pub = self.create_publisher(Path, '/mpc/predicted_path', 10)

        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info(f"CasADi MPC Node uruchomiony (N={self.N}, dt={self.dt}s, L={self.L}m)")

    def setup_mpc(self):
        # Wymiary stanu: [x, y, psi, v], sterowania: [a, delta]
        nx = 4
        nu = 2

        X = ca.SX.sym('X', nx, self.N + 1)
        U = ca.SX.sym('U', nu, self.N)
        P = ca.SX.sym('P', nx + self.N * nx) # [x0, y0, psi0, v0, xref_0..N-1]

        cost = 0
        g = []

        # Wagi funkcji celu
        w_pos = 20.0
        w_yaw = 5.0
        w_v = 2.0
        w_steer = 1.0
        w_dsteer = 5.0
        w_accel = 0.5

        # Warunek początkowy
        g.append(X[:, 0] - P[0:nx])

        last_u_steer = 0.0
        for k in range(self.N):
            st = X[:, k]
            u = U[:, k]

            x_k, y_k, psi_k, v_k = st[0], st[1], st[2], st[3]
            a_k, delta_k = u[0], u[1]

            # Model kinematyczny Ackermanna (Euler)
            x_next = x_k + v_k * ca.cos(psi_k) * self.dt
            y_next = y_k + v_k * ca.sin(psi_k) * self.dt
            psi_next = psi_k + (v_k / self.L) * ca.tan(delta_k) * self.dt
            v_next = v_k + a_k * self.dt

            st_next = ca.vertcat(x_next, y_next, psi_next, v_next)
            g.append(X[:, k + 1] - st_next)

            # Referencja w kroku k
            ref_idx = nx + k * nx
            x_ref = P[ref_idx]
            y_ref = P[ref_idx + 1]
            psi_ref = P[ref_idx + 2]
            v_ref = P[ref_idx + 3]

            cost += w_pos * ((x_k - x_ref)**2 + (y_k - y_ref)**2)
            cost += w_yaw * (ca.sin(psi_k - psi_ref))**2
            cost += w_v * (v_k - v_ref)**2
            cost += w_accel * (a_k**2)
            cost += w_steer * (delta_k**2)
            if k == 0:
                cost += w_dsteer * (delta_k - self.last_delta)**2
            else:
                cost += w_dsteer * (delta_k - U[1, k - 1])**2

        # Koszt terminalny
        ref_end_idx = nx + (self.N - 1) * nx
        cost += w_pos * 2.0 * ((X[0, self.N] - P[ref_end_idx])**2 + (X[1, self.N] - P[ref_end_idx + 1])**2)

        # Ograniczenia zmiennych optymalizacji
        opt_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp_prob = {'f': cost, 'x': opt_variables, 'g': ca.vertcat(*g), 'p': P}

        opts = {
            'ipopt.max_iter': 50,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-4,
            'ipopt.acceptable_obj_change_tol': 1e-4
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # Granice ograniczeń
        self.n_vars = opt_variables.size1()
        self.n_constraints = (self.N + 1) * nx

        # Zakresy dla X: [-inf, inf] dla x,y,psi, [v_min, v_max] dla v
        # Zakresy dla U: a in [-1.0, 1.0], delta in [-delta_max, delta_max]
        self.lbx = []
        self.ubx = []

        for _ in range(self.N + 1):
            self.lbx.extend([-ca.inf, -ca.inf, -ca.inf, self.v_min])
            self.ubx.extend([ca.inf, ca.inf, ca.inf, self.v_max])

        for _ in range(self.N):
            self.lbx.extend([-1.0, -self.delta_max]) # [a_min, delta_min]
            self.ubx.extend([1.0, self.delta_max])   # [a_max, delta_max]

        self.lbg = [0.0] * self.n_constraints
        self.ubg = [0.0] * self.n_constraints

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_v = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.odom_received = True

    def plan_callback(self, msg: Path):
        self.path_points = []
        for p in msg.poses:
            self.path_points.append([p.pose.position.x, p.pose.position.y])
        self.get_logger().info(f"Odebrano nową ścieżkę A* ({len(self.path_points)} punktów)")

    def control_loop(self):
        if not self.odom_received or len(self.path_points) < 2:
            return

        # Sprawdzenie czy osiągnięto cel
        goal_x, goal_y = self.path_points[-1]
        dist_to_goal = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
        if dist_to_goal < self.goal_tol:
            self.stop_robot()
            return

        # Znalezienie najbliższego punktu na ścieżce
        dists = [math.hypot(p[0] - self.current_x, p[1] - self.current_y) for p in self.path_points]
        min_idx = int(np.argmin(dists))

        # Przygotowanie wektora parametrów referencji w horyzoncie N
        p_val = [self.current_x, self.current_y, self.current_yaw, self.current_v]

        idx = min_idx
        v_target = self.v_max
        step_dist = max(v_target * self.dt, 0.05)

        accum_dist = 0.0
        curr_p = self.path_points[idx]

        for k in range(self.N):
            target_dist = (k + 1) * step_dist
            while idx < len(self.path_points) - 1:
                seg_d = math.hypot(self.path_points[idx+1][0] - self.path_points[idx][0],
                                   self.path_points[idx+1][1] - self.path_points[idx][1])
                if accum_dist + seg_d >= target_dist:
                    break
                accum_dist += seg_d
                idx += 1

            pt = self.path_points[min(idx, len(self.path_points) - 1)]
            pt_next = self.path_points[min(idx + 1, len(self.path_points) - 1)]
            ref_yaw = math.atan2(pt_next[1] - pt[1], pt_next[0] - pt[0]) if idx < len(self.path_points) - 1 else self.current_yaw

            p_val.extend([pt[0], pt[1], ref_yaw, v_target])

        # Rozwiązanie zadania optymalizacji
        try:
            x0 = [0.0] * self.n_vars
            sol = self.solver(x0=x0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=p_val)
            res = sol['x'].full().flatten()

            # Odczyt pierwszego kroku sterowania: U[:, 0]
            # U zaczyna się po (N + 1) * 4 zmiennych stanu
            u_start_idx = (self.N + 1) * 4
            a_cmd = res[u_start_idx]
            delta_cmd = res[u_start_idx + 1]

            # Wyliczenie zadanej prędkości liniowej
            v_cmd = np.clip(self.current_v + a_cmd * self.dt, self.v_min, self.v_max)
            self.last_delta = delta_cmd

            # Publikacja /cmd_vel
            cmd_msg = Twist()
            cmd_msg.linear.x = float(v_cmd)
            cmd_msg.angular.z = float(delta_cmd)
            self.cmd_pub.publish(cmd_msg)

            # Publikacja przewidywanej trajektorii MPC
            self.publish_predicted_path(res)

        except Exception as e:
            self.get_logger().error(f"Błąd solvera CasADi: {e}")
            self.stop_robot()

    def publish_predicted_path(self, opt_res):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        for k in range(self.N + 1):
            px = opt_res[k * 4]
            py = opt_res[k * 4 + 1]
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.position.z = 0.0
            path_msg.poses.append(ps)

        self.pred_path_pub.publish(path_msg)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
