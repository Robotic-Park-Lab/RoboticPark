import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String


class TriangulatedFormationNode(Node):
    """3D triangulated affine formation control law from "Distributed
    control of triangulated 3D multirobot formations with scale
    flexibility": a Delaunay-triangulated mesh of reference positions
    r_i (eq 1-8), driven by two leader robots that fix translation,
    rotation and scale (eqs 46-55, Theorem 2). Followers converge
    exponentially to the affine-transformed mesh under

        u_F = -(A_FL @ p_L + A_FF @ p_F)                          (eq 55)

    with A = sum_m S_m^T P_m S_m (eq 11) computed once at startup from
    the mesh's reference positions and triangulation -- both entirely
    data-driven (read from the experience config file's `Robots`
    `role: leader`/`role: follower` + `pose` and its `Mesh.triangles`
    section), so the same node runs for any mesh satisfying Assumption 1
    (no two triangle-sharing vertices sharing the same reference z) and
    Assumption 2 (the two leaders' C_L = [C1;C2] is nonsingular, which
    in particular requires the two leaders to have different reference
    z -- see the module docstring in the experience config for the
    concrete mesh used in each experiment).

    Leaders are NOT commanded by this node: their target_pose is driven
    externally by the Supervisor (see the experience's topics.yaml),
    exactly like the sheep in AffineHerdingNode. This node only
    publishes target_pose for the followers.
    """

    def __init__(self):
        super().__init__('triangulated_formation_node')

        self.declare_parameter('config_file', 'path')
        self.declare_parameter('control_period', 0.1)
        self.declare_parameter('control_gain', 1.0)
        # Prop 3 (eq 29) actuation-uncertainty injection, disabled by
        # default: p_dot_i = c_i R_i u_i instead of p_dot_i = u_i, with
        # c_i in [uncertainty_gain_min, uncertainty_gain_max] and a
        # z-axis rotation of at most uncertainty_angle_max_deg, drawn
        # once per follower at startup and held fixed for the run (a
        # piecewise-constant instance of the piecewise-continuous c_i(t),
        # R_i(t) the theorem allows).
        self.declare_parameter('uncertainty_enable', False)
        self.declare_parameter('uncertainty_gain_min', 0.8)
        self.declare_parameter('uncertainty_gain_max', 1.2)
        self.declare_parameter('uncertainty_angle_max_deg', 20.0)
        self.declare_parameter('uncertainty_seed', 42)

        self.tau = self.get_parameter('control_period').value
        self.control_gain = self.get_parameter('control_gain').value
        self.uncertainty_enable = self.get_parameter('uncertainty_enable').value

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        with open(config_file, 'r') as f:
            documents = yaml.safe_load(f)

        # =========================
        # Mesh discovery (role-based; leaders first, C_L = [C1;C2] needs
        # a fixed order matching eq 46)
        # =========================
        robots_cfg = documents['Robots']
        self.leader_names = [r['name'] for r in robots_cfg.values() if r.get('role') == 'leader']
        self.follower_names = [
            r['name'] for r in robots_cfg.values() if r.get('role') == 'follower']
        if len(self.leader_names) != 2:
            raise ValueError(
                f"Need exactly 2 robots with role: leader, found {len(self.leader_names)}")
        if len(self.follower_names) < 2:
            raise ValueError(
                f"Need at least 2 robots with role: follower, found {len(self.follower_names)}")

        self.names = self.leader_names + self.follower_names
        self.N = len(self.names)
        name_to_robot = {r['name']: r for r in robots_cfg.values()}
        ref_positions = []
        for name in self.names:
            pose = name_to_robot[name]['pose']
            if isinstance(pose, str):
                pose = [float(v) for v in pose.replace(',', ' ').split()]
            ref_positions.append(pose)
        self.r = np.array(ref_positions, dtype=float)  # (N,3), reference mesh r_i

        triangles_1idx = documents['Mesh']['triangles']
        name_order = {name: i for i, name in enumerate(self.names)}
        # Mesh.triangles indexes robots by their position in the config's
        # Robots section (1-indexed, RobotNN order), independent of role
        # ordering -- resolve via the RobotNN -> name mapping.
        robot_key_order = list(robots_cfg.keys())
        idx_to_name = {i + 1: robots_cfg[key]['name'] for i, key in enumerate(robot_key_order)}
        self.triangles = [
            tuple(sorted(name_order[idx_to_name[t]] for t in tri)) for tri in triangles_1idx
        ]
        if len(self.triangles) < 1:
            raise ValueError("Mesh.triangles is empty")

        # =========================
        # Offline design stage (Section IV-A.1): C_i (eq 3), A (eq 11),
        # and its leader/follower partition (eq 47) -- computed once.
        # =========================
        self.C = [self._C_i(self.r[i]) for i in range(self.N)]
        self.A = self._build_A()
        self.A_LL = self.A[0:6, 0:6]
        self.A_LF = self.A[0:6, 6:]
        self.A_FL = self.A[6:, 0:6]
        self.A_FF = self.A[6:, 6:]

        eig_FF = np.linalg.eigvalsh(self.A_FF)
        if eig_FF[0] < 1e-6:
            self.get_logger().warn(
                'A_FF smallest eigenvalue is %.3e -- Assumption 1/2 may be marginally '
                'satisfied for this mesh, expect slow convergence.' % eig_FF[0])

        if self.uncertainty_enable:
            gmin = self.get_parameter('uncertainty_gain_min').value
            gmax = self.get_parameter('uncertainty_gain_max').value
            amax = np.deg2rad(self.get_parameter('uncertainty_angle_max_deg').value)
            seed = self.get_parameter('uncertainty_seed').value
            rng = np.random.default_rng(seed)
            self.c_gain = rng.uniform(gmin, gmax, size=len(self.follower_names))
            self.R_rot = []
            for _ in self.follower_names:
                theta = rng.uniform(-amax, amax)
                ct, st = np.cos(theta), np.sin(theta)
                self.R_rot.append(np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]]))

        # =========================
        # State / ROS I/O
        # =========================
        self.positions = {name: None for name in self.names}
        self.formation = False

        self.create_subscription(String, '/swarm/order', self.order_callback, 10)
        for name in self.names:
            self.create_subscription(
                PoseStamped, f'/{name}/local_pose',
                lambda msg, n=name: self.pose_callback(msg, n), 10)
        self.targets = {
            name: self.create_publisher(PoseStamped, f'/{name}/target_pose', 10)
            for name in self.follower_names
        }
        self.pub_error = self.create_publisher(Float64, '/formation_error', 10)
        self.pub_scale = self.create_publisher(Float64, '/formation_scale', 10)

        self.timer = self.create_timer(self.tau, self.update)
        self.get_logger().info(
            'Triangulated Formation Node started. N=%d (leaders: %s, %d followers), '
            'M=%d triangles, uncertainty=%s' % (
                self.N, str(self.leader_names), len(self.follower_names),
                len(self.triangles), self.uncertainty_enable))

    # =====================================================
    def order_callback(self, msg):
        if msg.data == 'formation_run':
            self.formation = True
        elif msg.data == 'formation_stop':
            self.formation = False

    def pose_callback(self, msg, name):
        self.positions[name] = msg

    # =====================================================
    @staticmethod
    def _C_i(ri):
        """eq 3."""
        x, y, z = ri
        return np.array([
            [x, -y, 0.0, 1.0, 0.0, 0.0],
            [y, x, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, z, 0.0, 0.0, 1.0],
        ])

    def _build_A(self):
        """eq 4, 8, 11: A = sum_m S_m^T P_m S_m, assembled by scattering
        each triangle's 9x9 P_m into the global 3N x 3N matrix (equivalent
        to the selector-matrix form, without materializing S_m)."""
        A = np.zeros((3 * self.N, 3 * self.N))
        for tri in self.triangles:
            Z = np.vstack([self.C[i] for i in tri])       # (9,6)
            P = np.eye(9) - Z @ np.linalg.pinv(Z)          # eq 8
            for a, i in enumerate(tri):
                for b, j in enumerate(tri):
                    A[3 * i:3 * i + 3, 3 * j:3 * j + 3] += P[3 * a:3 * a + 3, 3 * b:3 * b + 3]
        return A

    # =====================================================
    def update(self):
        if not self.formation:
            return
        if any(self.positions[n] is None for n in self.names):
            return

        p = np.array([
            [self.positions[n].pose.position.x,
             self.positions[n].pose.position.y,
             self.positions[n].pose.position.z]
            for n in self.names
        ]).flatten()  # (3N,)

        pL = p[0:6]
        pF = p[6:]

        uF = -self.control_gain * (self.A_FL @ pL + self.A_FF @ pF)  # eq 55

        if self.uncertainty_enable:
            uF = self._apply_uncertainty(uF)

        pF_next = pF + self.tau * uF

        for k, name in enumerate(self.follower_names):
            x, y, z = pF_next[3 * k:3 * k + 3]
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            self.targets[name].publish(msg)

        self._publish_diagnostics(p)

    def _apply_uncertainty(self, uF):
        """eq 29 (Prop 3): p_dot_i = c_i R_i u_i."""
        out = np.zeros_like(uF)
        for k in range(len(self.follower_names)):
            ui = uF[3 * k:3 * k + 3]
            out[3 * k:3 * k + 3] = self.c_gain[k] * (self.R_rot[k] @ ui)
        return out

    def _publish_diagnostics(self, p):
        Ap = self.A @ p
        err = Float64()
        err.data = float(np.linalg.norm(Ap))  # ||Ap|| -> 0 at convergence (Thm 1/2)
        self.pub_error.publish(err)

        # eq 54: v = C_L^-1 p_L (the two leaders fully determine
        # translation/rotation/scale). Publish the x-y scale magnitude
        # sqrt(a^2+b^2) for logging/plots of the scale-flexibility claim.
        C_L = np.vstack([self.C[0], self.C[1]])
        try:
            v = np.linalg.solve(C_L, p[0:6])
            scale = Float64()
            scale.data = float(np.hypot(v[0], v[1]))
            self.pub_scale.publish(scale)
        except np.linalg.LinAlgError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TriangulatedFormationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
