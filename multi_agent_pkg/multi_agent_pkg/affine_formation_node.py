import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String


class AffineHerdingNode(Node):
    """Herding control law from "Distributed formation control for
    encirclement and herding" (Garcia-Lechuz, Manas-Alvarez, Aragues,
    Guinaldo, Lopez-Nicolas): herders arranged in a cycle graph drive a
    purely reactive "sheep" agent toward a target region by continuously
    opening/closing a circular arc (complex-Laplacian control, eqs.
    15-16, 26-27, 40-45 of the paper).

    N herders and the sheep are discovered from the experience config
    file's `Robots` section by `role` (`herder` / `herd`), not hardcoded
    -- the same node runs unmodified for N=5, 7, 9, 13, ...
    """

    def __init__(self):
        super().__init__('affine_herding_node')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('config_file', 'path')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('theta_epsilon', 0.1)  # eq 40: theta_min = pi + epsilon
        self.declare_parameter('control_gain', 1.0)   # explicit, tunable replacement
        # for the ad hoc "3*"/"6*" scaling that used to be hardcoded into
        # the control law below.

        self.radius = self.get_parameter('radius').value
        self.theta_epsilon = self.get_parameter('theta_epsilon').value
        self.control_gain = self.get_parameter('control_gain').value

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)

        # =========================
        # Herder/sheep discovery (role-based, not name-pattern-based)
        # =========================
        self.sheep_name = None
        self.herders_list = []
        for robot in documents['Robots'].values():
            role = robot.get('role')
            name = robot['name']
            if role == 'herder':
                self.herders_list.append(name)
            elif role == 'herd':
                self.sheep_name = name
        if self.sheep_name is None:
            raise ValueError("No robot with role: herd found in config_file's Robots section")
        if len(self.herders_list) < 3:
            raise ValueError(
                f"Need at least 3 herders (role: herder), found {len(self.herders_list)}")
        self.N = len(self.herders_list)

        # =========================
        # Internal state
        # =========================
        self.sheep_position = PoseStamped()
        self.goal_position = PoseStamped()
        self.herder_positions = {}
        self.herders = {}
        self.status = False
        self.formation = False
        self.check_zone = False
        self.check_order = False
        self.dist_max = 0.0

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(
            PoseStamped, f'/{self.sheep_name}/local_pose', self.sheep_callback, 10)
        self.create_subscription(
            PoseStamped, f'/{self.sheep_name}/target_pose', self.goal_callback, 10)
        self.create_subscription(String, '/swarm/order', self.order_callback, 10)

        for name in self.herders_list:
            self.create_subscription(
                PoseStamped,
                f'/{name}/local_pose',
                lambda msg, n=name: self.herder_callback(msg, n),
                10)
            self.herders[name] = self.create_publisher(PoseStamped, f'/{name}/target_pose', 10)

        self.pub_centroid = self.create_publisher(PoseStamped, '/virtual_centroid', 10)
        self.pub_theta = self.create_publisher(Float64, '/current_theta', 10)
        self.pub_dist = self.create_publisher(Float64, '/dist_sp', 10)

        self.initialize()
        self.timer = self.create_timer(self.tau, self.update)
        self.get_logger().info(
            'Affine Herding Node started. N=%d herders: %s, sheep: %s' %
            (self.N, str(self.herders_list), self.sheep_name))

    def initialize(self):
        self.theta_min = np.pi + self.theta_epsilon
        self.theta_max = 2 * np.pi - 2 * np.pi / self.N  # eq 16 upper bound
        self.diff_theta = self.theta_max - self.theta_min
        self.theta_it = self.theta_max

        self.tau = 0.1  # control period (s)
        self.J = np.array([[0, -1], [1, 0]])  # eq 42

        self.check_init = True

    # =====================================================
    # Callbacks
    # =====================================================
    def order_callback(self, msg):
        if msg.data == 'formation_run':
            self.formation = True
        elif msg.data == 'formation_stop':
            self.formation = False

    def sheep_callback(self, msg):
        self.sheep_position = msg

    def goal_callback(self, msg):
        # `goal_position` plays the role of the safe-region center `c` in
        # eqs. 5, 40, 43: the Supervisor publishes it once at the start of
        # the experiment via <sheep>/target_pose (see the experience's
        # topics.yaml).
        self.goal_position = msg
        delta = np.array([
            self.sheep_position.pose.position.x - self.goal_position.pose.position.x,
            self.sheep_position.pose.position.y - self.goal_position.pose.position.y
        ])
        self.dist_max = np.linalg.norm(delta)
        self.check_order = False
        self.status = True

    def herder_callback(self, msg, name):
        self.herder_positions[name] = msg

    # =====================================================
    # Main update
    # =====================================================
    def update(self):
        if not self.status or not self.formation:
            return
        if len(self.herder_positions) < self.N:
            return

        if not self.check_order:
            self.opening_index, self.ordered_names = self.compute_opening_between()
            self.get_logger().info(
                'Opening index: %s. Herder order: %s' %
                (str(self.opening_index), str(self.ordered_names)))
            self.check_order = True

        c = np.array([self.goal_position.pose.position.x, self.goal_position.pose.position.y])
        mu = self.herder_centroid()
        self.dist_sp = np.linalg.norm(mu - c)

        # eq 40: omega = tanh(||mu-c||^2 / ||mu(0)-c||^2), theta interpolates
        # theta_max (fully open) toward theta_min (fully closed) as the
        # herd centroid nears the target region.
        denom = max(self.dist_max ** 2, 1e-6)
        omega = np.tanh((self.dist_sp ** 2) / denom)
        theta_it = self.theta_max if self.dist_sp < 0.05 else \
            self.theta_max - omega * self.diff_theta
        self.theta_it = theta_it

        self.get_logger().info(
            'dist: %.3f theta: %.3f' % (self.dist_sp, theta_it), throttle_duration_sec=0.5)

        A, _ = self.find_prp_gains_2d(self.N, theta_it)
        B, _ = self.find_t_gains_2d(2 * np.pi - theta_it)
        d = 2 * self.radius * np.sin((2 * np.pi - theta_it) / 2)  # eq 41

        # Safe-region switch (eq 5, Sρ(r)): once the herd centroid is
        # within the safe radius of the target, use the target as the
        # control center; otherwise use the herd's own centroid so the
        # arc travels with the herd toward the target.
        if self.check_zone:
            center = c if self.dist_sp < self.radius else mu
            self.check_zone = self.dist_sp < self.radius
        else:
            center = mu if self.dist_sp > self.radius * 0.7 else c
            self.check_zone = self.dist_sp <= self.radius * 0.7

        targets = self.distributed_formation_control(A, B, d, center, mu, c)
        self.publish_outputs(targets, theta_it, mu)

    # =====================================================
    # Herd centroid (eq: mu_dot = mean(u_i), so mu(t) = mean(q_i(t))
    # exactly, by construction -- no separate integration needed).
    # =====================================================
    def herder_centroid(self):
        pts = np.array([
            [self.herder_positions[n].pose.position.x, self.herder_positions[n].pose.position.y]
            for n in self.herders_list
        ])
        return pts.mean(axis=0)

    # =====================================================
    # Opening EXACTLY between two herders, ordered by angle around the
    # herd centroid, rotated so the gap sits after ordered_names[-1] and
    # before ordered_names[0]. Also captures mu(0), q1(0), qn(0) for the
    # rotation term (eq 43).
    # =====================================================
    def compute_opening_between(self):
        mu0 = self.herder_centroid()
        goal = np.array([self.goal_position.pose.position.x, self.goal_position.pose.position.y])

        direction = goal - mu0
        self.goal_angle = np.arctan2(direction[1], direction[0])

        name_angle_list = []
        for name, pose in self.herder_positions.items():
            rel = np.array([pose.pose.position.x, pose.pose.position.y]) - mu0
            ang = np.arctan2(rel[1], rel[0])
            if ang < 0:
                ang += 2 * np.pi
            name_angle_list.append((name, ang))

        name_angle_list.sort(key=lambda x: x[1])
        ordered_names = [x[0] for x in name_angle_list]
        ordered_angles = np.array([x[1] for x in name_angle_list])

        diffs = self.wrap_angle(ordered_angles - self.goal_angle)
        positive_values = diffs[diffs > 0]
        idx = np.where(diffs == np.min(positive_values))[0][0]

        ordered_names = ordered_names[-(self.N - idx):] + ordered_names[:-(self.N - idx)]

        # eq 43 denominator terms, captured once at formation start (t=0)
        self.mu0 = mu0
        self.c0 = goal
        self.q0 = np.zeros((self.N, 2))
        for agent_idx, name in enumerate(ordered_names):
            pose = self.herder_positions[name]
            self.q0[agent_idx, 0] = pose.pose.position.x
            self.q0[agent_idx, 1] = pose.pose.position.y

        return idx, ordered_names

    # =====================================================
    # Publish
    # =====================================================
    def publish_outputs(self, targets, theta, mu):
        for name, point in targets.items():
            if name not in self.herders:
                continue
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            here = self.herder_positions[name].pose.position
            pose.pose.position.x = here.x + 0.5 * (float(point[0]) - here.x)
            pose.pose.position.y = here.y + 0.5 * (float(point[1]) - here.y)
            pose.pose.position.z = 1.0
            self.herders[name].publish(pose)

        centroid_msg = PoseStamped()
        centroid_msg.header.stamp = self.get_clock().now().to_msg()
        centroid_msg.header.frame_id = "map"
        centroid_msg.pose.position.x = float(mu[0])
        centroid_msg.pose.position.y = float(mu[1])
        self.pub_centroid.publish(centroid_msg)

        theta_msg = Float64()
        theta_msg.data = float(theta)
        self.pub_theta.publish(theta_msg)
        dist_msg = Float64()
        dist_msg.data = float(self.dist_sp)
        self.pub_dist.publish(dist_msg)

    # =====================================================
    # Control law: interior herders use eq 44 (uf), boundary herders
    # (the two ends of the open arc) use eq 45 (ul), integrated with a
    # fixed step self.tau.
    # =====================================================
    def distributed_formation_control(self, A, B, d, center, mu, c):
        N = self.N
        names = self.ordered_names

        qmat = np.zeros((N + 1, 2))
        for i, name in enumerate(names):
            pose = self.herder_positions[name]
            qmat[i] = [pose.pose.position.x, pose.pose.position.y]
        qmat[N] = np.array([
            self.sheep_position.pose.position.x, self.sheep_position.pose.position.y])

        delta_1n = qmat[0] - qmat[N - 1]
        d_1n = np.linalg.norm(delta_1n)

        dq = np.zeros((N, 2))

        # Rotation term (eq 43), shared by both boundary agents.
        w = qmat[0] - qmat[N - 1]                       # (q1 - qn)
        v = mu - c                                       # (mu - c)
        num = np.inner(v, w)
        den = np.linalg.norm(self.mu0 - self.c0) * np.linalg.norm(self.q0[0] - self.q0[N - 1])
        Omega = num / den if den > 1e-6 else 0.0

        for agent in range(N):
            if 0 < agent < N - 1:
                # ---- interior herders: eq 44 ----
                for neighbor in (agent - 1, agent + 1):
                    Aij = A[2 * agent:2 * agent + 2, 2 * neighbor:2 * neighbor + 2]
                    dq[agent] += self.control_gain * Aij @ (qmat[neighbor] - qmat[agent])
            else:
                # ---- boundary herders (arc ends): eq 45 ----
                B31 = B[4:6, 0:2]
                B32 = B[4:6, 2:4]
                B21 = B[2:4, 0:2]
                B23 = B[2:4, 4:6]

                if agent == 0:
                    dq[agent] += self.control_gain * B32 @ (qmat[N - 1] - qmat[agent])
                    dq[agent] += self.control_gain * B31 @ (qmat[N] - qmat[agent])
                    dq[agent] += -(d_1n ** 2 - d ** 2) * (qmat[agent] - qmat[N - 1])
                else:  # agent == N - 1
                    dq[agent] += self.control_gain * B23 @ (qmat[0] - qmat[agent])
                    dq[agent] += self.control_gain * B21 @ (qmat[N] - qmat[agent])
                    dq[agent] += -(d_1n ** 2 - d ** 2) * (qmat[agent] - qmat[0])

                dq[agent] += Omega * (self.J @ (qmat[agent] - mu))

        qmat[:N] += self.tau * dq

        return {names[i]: np.array([qmat[i, 0], qmat[i, 1], 0.75]) for i in range(N)}

    def find_prp_gains_2d(self, N, theta):
        """eqs 15-16: A_deg matrix (complex Laplacian, realified) for the N-1
        interior/relative gains along the open cycle."""
        adj = np.zeros((N, N))
        for i in range(N):
            j = (i + 1) % N
            adj[i, j] = 1
            adj[j, i] = 1

        t1 = 1.0 / np.tan(theta / (2 * (N - 1)))
        t2 = 1.0 / np.tan(-theta / 2)

        A = np.zeros((2 * N, 2 * N))
        for agent in range(N):
            v = 0
            for neighbor in range(N):
                if adj[agent, neighbor] == 1:
                    if (agent == 0 and neighbor == N - 1) or (agent == N - 1 and neighbor == 0):
                        Aij = np.array([[t2, 1], [-1, t2]])
                    elif agent == 0 or agent == N - 1:
                        Aij = np.array([[t1, 1], [-1, t1]])
                    else:
                        Aij = np.array([[t1, -1], [1, t1]])
                    if v == 1:
                        Aij[0, 1] *= -1
                        Aij[1, 0] *= -1
                    A[2 * agent:2 * agent + 2, 2 * neighbor:2 * neighbor + 2] = Aij
                    v += 1
        A = A - np.diag(np.sum(A, axis=1))
        eigvals = np.sort(np.linalg.eig(A)[0])
        return A, eigvals[0].real

    def find_t_gains_2d(self, theta):
        """eqs 26-27: A_tri matrix (isosceles triangle {sheep, qn, q1})."""
        N = 3
        t1 = np.tan(theta / 2.0)
        t2 = 1.0 / np.tan(theta)
        A = np.zeros((2 * N, 2 * N))
        A[0:2, 2:4] = np.array([[t1, 1], [-1, t1]])
        A[2:4, 0:2] = np.array([[t1, -1], [1, t1]])
        A[0:2, 4:6] = np.array([[t1, -1], [1, t1]])
        A[4:6, 0:2] = np.array([[t1, 1], [-1, t1]])
        A[2:4, 4:6] = np.array([[t2, 1], [-1, t2]])
        A[4:6, 2:4] = np.array([[t2, -1], [1, t2]])
        A = A - np.diag(np.sum(A, axis=1))
        eigvals = np.sort(np.real(np.linalg.eigvals(A)))
        return A, eigvals[0]

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = AffineHerdingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
