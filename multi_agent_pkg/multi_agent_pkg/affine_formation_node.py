import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String


class AffineHerdingNode(Node):

    def __init__(self):
        super().__init__('affine_herding_node')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('num_agents', 13)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('diff_theta_deg', 60.0)
        self.declare_parameter('lambda_filter_gain', 1.0)
        self.declare_parameter('allow_full_closure', True)
        self.declare_parameter('config_file', 'path')

        self.N = self.get_parameter('num_agents').value
        self.radius = self.get_parameter('radius').value
        self.diff_theta = np.deg2rad(self.get_parameter('diff_theta_deg').value)
        self.lambda_gain = self.get_parameter('lambda_filter_gain').value
        self.allow_full_closure = self.get_parameter('allow_full_closure').value

        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)

        # =========================
        # Internal state
        # =========================
        self.agent_position = PoseStamped()
        self.goal_position = PoseStamped()
        self.herder_positions = {}
        self.herders = {}
        self.herders_list = []
        self.status = False
        self.formation = False
        self.check_order = False
        self.check_init = False
        self.dist_max = 0.0
        self.control_type = True # False=Simple; True=IROS

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(PoseStamped, '/khepera01/local_pose', self.agent_callback, 10)
        self.create_subscription(PoseStamped, '/khepera01/target_pose', self.goal_callback, 10)
        self.create_subscription(String, '/swarm/order', self.order_callback, 10)

        for robot in documents['Robots']:
            name = documents['Robots'][robot]['name']
            if 'dron' in name:
                self.create_subscription(
                    PoseStamped, 
                    f'/{name}/local_pose', 
                    lambda msg, n=name: self.herder_callback(msg, n), 
                    10)
                self.herders[name] = self.create_publisher(
                    PoseStamped,
                    f'/{name}/target_pose', 
                    10)
                self.herders_list.append(name)

        self.pub_centroid = self.create_publisher(PoseStamped, '/virtual_centroid', 10)
        self.pub_agent = self.create_publisher(PoseStamped, '/khepera01/target_pose', 10)
        self.pub_theta = self.create_publisher(Float64, '/current_theta', 10)
        self.pub_dist = self.create_publisher(Float64, '/dist_sp', 10)

        self.initialize()
        self.timer = self.create_timer(self.tau, self.update)
        self.get_logger().info("Affine Herding Node Started")

    def initialize(self):
        self.theta_min = np.pi + 0.8
        self.theta_max = 2*np.pi - 2*np.pi/self.N
        self.theta_it = self.theta_max

        self.A, self.k_a = self.find_prp_gains_2d(self.N, self.theta_min)
        self.B, self.k_b = self.find_t_gains_2d(2*np.pi-self.theta_min)

        self.d = 2*self.radius*np.sin((2*np.pi-self.theta_max)/2)
        self.k_c = np.min([self.k_a, self.k_b])

        self.alpha = 0.8
        self.tau = self.alpha*(-1.0/self.k_c)
        self.diff_theta = self.theta_max - self.theta_min
        self.J = np.array([[0, -1], [1, 0]])

        # Grafo
        self.adj = np.zeros((self.N, self.N))
        for i in range(self.N-1):
            j = np.mod(i, self.N) + 1
            self.adj[i,j] = 1
            self.adj[j,i] = 1
        
        self.get_logger().info('\nadj:%s \nka: %.2f kb: %.2f kc: %.2f tau: %.2f' % (str(self.adj), self.k_a, self.k_b, self.k_c, self.tau))
        

        self.check_init = True

    # =====================================================
    # Callbacks
    # =====================================================
    def order_callback(self, msg):
        if msg.data == 'formation_run':
            self.formation = True
        elif msg.data == 'formation_stop':
            self.formation = False

    def agent_callback(self, msg):
        self.agent_position = msg

    def goal_callback(self, msg):
        self.goal_position = msg
        delta = np.array([
            self.agent_position.pose.position.x - self.goal_position.pose.position.x,
            self.agent_position.pose.position.y - self.goal_position.pose.position.y
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
        # =====================================================
        # Comprobaciones iniciales
        # =====================================================
        if not self.status or not self.formation:
            return

        if len(self.herder_positions) < self.N:
            return

        if not self.check_order:
            self.opening_index, self.ordered_names = self.compute_opening_between()
            self.get_logger().info('IDX: %s. Orden: %s' % (str(self.opening_index), str(self.ordered_names)))
            self.check_order = True

        # =====================================================
        # Bucle principal
        # =====================================================
        delta = np.array([
            self.agent_position.pose.position.x - self.goal_position.pose.position.x, 
            self.agent_position.pose.position.y - self.goal_position.pose.position.y
            ])
        self.dist_sp = np.linalg.norm(delta)

        self.omega = np.min([1.0, self.dist_sp/self.dist_max])

        if self.dist_sp<0.14:
            theta_it = self.theta_max
        else:
            theta_it = self.theta_max - self.omega * self.diff_theta
        
        self.dtheta_dt = (theta_it - self.theta_it)/self.tau 
        self.theta_it = theta_it

        self.get_logger().info('dist: %.3f theta: %.3f' % (self.dist_sp, theta_it), throttle_duration_sec=0.5)

        # =====================================================
        # Ley de control
        # =====================================================
        if self.control_type:
            # Actualizamos ganancias y pasos
            self.A, aux = self.find_prp_gains_2d(self.N, self.theta_it)
            self.B, aux = self.find_t_gains_2d(2*np.pi-self.theta_it)
            self.d = 2*self.radius*np.sin((2*np.pi-self.theta_it)/2)
            # Centro de seguridad
            if self.dist_sp>0.14:
                center=self.agent_position.pose
            else:
                center=self.goal_position.pose
            ###################################
            targets = self.distributed_formation_control(
                self.A,
                self.B,
                self.d,
                np.array([
                    center.position.x,
                    center.position.y
                ])
            )
            self.publish_outputs(targets, theta_it)
        
        if not self.control_type:
            # Centro de seguridad
            if self.dist_sp>0.14:
                center=self.agent_position.pose
            else:
                center=self.goal_position.pose
                theta_it = self.theta_max+0.4
            ###################################
            targets_arc = self.generate_arc(
                center=center,
                radius=self.radius,
                total_angle=theta_it,
                opening_index=self.opening_index,
                ordered_names=self.ordered_names
            )
                
            self.publish_outputs(targets_arc, theta_it)

    # =====================================================
    # Apertura EXACTAMENTE entre dos drones
    # =====================================================
    def compute_opening_between(self):
        center = np.array([self.agent_position.pose.position.x, self.agent_position.pose.position.y])

        goal = np.array([self.goal_position.pose.position.x, self.goal_position.pose.position.y])

        direction = goal - center
        self.goal_angle = np.arctan2(direction[1], direction[0])
        self.get_logger().info('Goal Angle: %.3f' % self.goal_angle)
        name_angle_list = []

        for name, pose in self.herder_positions.items():
            pos = np.array([
                pose.pose.position.x,
                pose.pose.position.y
            ])

            rel = pos - center
            ang = np.arctan2(rel[1], rel[0])
            if ang<0:
                ang = ang + 2*np.pi

            name_angle_list.append((name, ang))

        
        # ORDENAR POR ÁNGULO REAL
        name_angle_list.sort(key=lambda x: x[1])

        ordered_names = [x[0] for x in name_angle_list]
        ordered_angles = np.array([x[1] for x in name_angle_list])

        # Encontrar el dron más alineado con el goal
        diffs = self.wrap_angle(ordered_angles - self.goal_angle)
        self.get_logger().debug('%s' % str(diffs))
        self.get_logger().debug('%s' % str(ordered_names))
        positive_values = diffs[diffs > 0]
        # idx = np.argmin(diffs)
        idx = np.where(diffs == np.min(positive_values))[0][0]

        ordered_names = ordered_names[-(self.N-idx):] + ordered_names[:-(self.N-idx)]

        self.s0 = center
        self.q0 = np.zeros((self.N, 2))
        for agent_idx in range(self.N):
            name = ordered_names[agent_idx]
            pose = self.herder_positions[name]
            self.q0[agent_idx, 0] = pose.pose.position.x
            self.q0[agent_idx, 1] = pose.pose.position.y

        return idx, ordered_names

    # =====================================================
    # Arc generator (gap between drones)
    # =====================================================
    def generate_arc(self, center, radius, total_angle, opening_index, ordered_names):
        N = len(ordered_names)

        # Ángulos uniformes
        angles = np.linspace(0, total_angle, N, endpoint=False)
        # angles = angles - total_angle/2
        self.get_logger().debug('angles init: %s.' % (str(angles)))
            
        # Desplazamiento para que el hueco quede ENTRE drones
        angle_step = total_angle / N
        offset = 2*np.pi - angles[N-1] # angle_step/2
        aux = angles + offset/2 + self.goal_angle
        self.get_logger().debug('angles init2: %s.' % (str(aux)))
        idx = self.herders_list.index(ordered_names[0])
        self.get_logger().debug('idx: %d.' % (idx))
        targets = {}

        for name, a in zip(ordered_names, aux):
            x = radius * np.cos(a)
            y = radius * np.sin(a)
            self.get_logger().debug('%s: %.3f. x: %.2f y:%.2f' % (name, a, x, y))
            targets[name] = np.array([
                center.position.x + x,
                center.position.y + y,
                0.75
            ])

        return targets

    # =====================================================
    # Publish
    # =====================================================
    def publish_outputs(self, targets, theta):
        for name, point in targets.items():
            if name not in self.herders:
                continue
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.herder_positions[name].pose.position.x + 0.5*(float(point[0])-self.herder_positions[name].pose.position.x)
            pose.pose.position.y = self.herder_positions[name].pose.position.y + 0.5*(float(point[1])-self.herder_positions[name].pose.position.y)
            pose.pose.position.z = 1.0
            self.get_logger().debug('%s (goal)::X: %.3f Y: %.3f' % (name, pose.pose.position.x, pose.pose.position.y))
            self.get_logger().debug('%s (pose)::X: %.3f Y: %.3f' % (name, self.herder_positions[name].pose.position.x, self.herder_positions[name].pose.position.y))
            self.herders[name].publish(pose)

        centroid_msg = PoseStamped()
        centroid_msg.header.stamp = self.get_clock().now().to_msg()
        centroid_msg.header.frame_id = "map"
        centroid_msg.pose = self.agent_position.pose
        self.pub_centroid.publish(centroid_msg)

        theta_msg = Float64()
        theta_msg.data = float(theta)
        self.pub_theta.publish(theta_msg)
        dist_msg = Float64()
        dist_msg.data = float(self.dist_sp)
        self.pub_dist.publish(dist_msg)

    # =====================================================
    def distributed_formation_control(self, A, B, d, pt):

        N = self.N
        names = self.ordered_names  # Orden consistente con PRP

        # =====================================
        # 1. Construir matriz de estados
        # =====================================
        qmat = np.zeros((N+1, 2))

        for i, name in enumerate(names):
            pose = self.herder_positions[name]
            qmat[i] = [
                pose.pose.position.x,
                pose.pose.position.y
            ]

        # Última fila → oveja
        qmat[N] = [
            self.agent_position.pose.position.x,
            self.agent_position.pose.position.y
        ]

        # =====================================
        # 2. Distancia entre líderes (1 y N)
        # =====================================
        delta_1n = qmat[0] - qmat[N-1]
        d_1n = np.linalg.norm(delta_1n)

        # =====================================
        # 3. Dinámica PRP
        # =====================================
        dq = np.zeros((N, 2))

        for agent in range(N):
            # ------------------------------------------------
            # INTERIORES (1 ... N-2)
            # ------------------------------------------------
            if 0 < agent < N-1:
                for neighbor in [agent-1, agent+1]:
                    Aij = A[
                        2*agent:2*agent+2,
                        2*neighbor:2*neighbor+2
                    ]
                    dq[agent] += 4*Aij @ (qmat[neighbor] - qmat[agent])

                self.get_logger().info('%d:: dq: %s' % ( agent, str(dq[agent])))
            # ------------------------------------------------
            # EXTREMO 0 (primer pastor)
            # ------------------------------------------------
            else:
                B12 = B[0:2, 2:4]
                B13 = B[0:2, 4:6]
                B31 = B[4:6, 0:2]
                B32 = B[4:6, 2:4]
                B21 = B[2:4, 0:2]
                B23 = B[2:4, 4:6]

                if agent == 0:
                    # Con vecino 1
                    dq[agent] += B32 @ (qmat[N-1] - qmat[agent])
                    # Con oveja
                    dq[agent] += B31 @ (qmat[N] - qmat[agent])
                    # Término de distancia entre extremos
                    dq_d = -(d_1n**2 - d**2) * (qmat[agent] - qmat[N-1])
                    dq[agent] += dq_d
                # ------------------------------------------------
                # EXTREMO N-1 (último pastor)
                # ------------------------------------------------
                elif agent == N-1:
                    # Con vecino N-2
                    dq[agent] += B23 @ (qmat[0] - qmat[agent])
                    # Con oveja
                    B12 = B[2:4, 4:6]
                    dq[agent] += B21 @ (qmat[N] - qmat[agent])
                    # Término de distancia
                    dq_d = -(d_1n**2 - d**2) * (qmat[agent] - qmat[0])
                    dq[agent] += dq_d
                
                # ------------------------------------------------
                # TÉRMINO DE ROTACIÓN (idéntico a MATLAB)
                # ------------------------------------------------
                w = qmat[0] - qmat[N-1]
                v = pt - qmat[N]
                num = np.inner(v, w)
                den = (
                    np.linalg.norm(self.s0-pt) *
                    np.linalg.norm(self.q0[0] - self.q0[N-1])
                )
                if den > 1e-6:
                    Omega = num / den
                else:
                    Omega = 0.0
                dq[agent] += 2* Omega * (self.J @ (qmat[agent] - qmat[N]))
        
                self.get_logger().info('%d:: dq: %s' % ( agent, str(dq[agent])))

        # =====================================
        # 4. Integración discreta (solo pastores)
        # =====================================
        qmat[:N] += 0.1*dq # self.tau * dq

        # =====================================
        # 5. Construir targets
        # =====================================
        targets = {}

        for i, name in enumerate(names):
            targets[name] = np.array([
                qmat[i, 0],
                qmat[i, 1],
                0.75
            ])

        return targets

    def find_prp_gains_2d(self, N, theta):
        adj = np.zeros((N, N))

        for i in range(N):
            j = (i + 1) % N
            adj[i, j] = 1
            adj[j, i] = 1

        t1 = 1.0 / np.tan(theta / (2*(N-1)))
        t2 = 1.0 / np.tan(-theta/2)

        A = np.zeros((2*N, 2*N))

        for agent in range(N):
            v = 0
            for neighbor in range(N):
                if adj[agent, neighbor] == 1:
                    if (agent == 0 and neighbor == N-1) or (agent == N-1 and neighbor == 0):
                        Aij = np.array([[t2, 1], [-1, t2]])
                    elif agent == 0 or agent == N-1:
                        Aij = np.array([[t1, 1], [-1, t1]])
                    else:
                        Aij = np.array([[t1, -1], [1, t1]])
                    if v == 1:
                        Aij[0,1] *= -1
                        Aij[1,0] *= -1
                    A[2*agent:2*agent+2, 2*neighbor:2*neighbor+2] = Aij
                    v += 1
        A = A - np.diag(np.sum(A, axis=1))
        eigvals = np.sort(np.linalg.eig(A)[0])
        k = eigvals[0].real
        return A, k
    
    def find_t_gains_2d(self, theta):
        """
        Devuelve:
            A  -> matriz Laplaciana 2N x 2N (con N = 3)
            k  -> autovalor mínimo (ordenado ascendente)
        """
        N = 3  # Fijo como en MATLAB
        # ===============================
        # 1) Construcción de ganancias
        # ===============================
        t1 = np.tan(theta / 2.0)
        t2 = 1.0 / np.tan(theta)  # cot(theta)
        A = np.zeros((2*N, 2*N))
        # ===============================
        # 2) Rellenar bloques exactamente como MATLAB
        # ===============================
        # A(1,2)
        A[0:2, 2:4] = np.array([[t1,  1],
                                [-1, t1]])
        # A(2,1)
        A[2:4, 0:2] = np.array([[t1, -1],
                                [ 1, t1]])
        # A(1,3)
        A[0:2, 4:6] = np.array([[t1, -1],
                                [ 1, t1]])
        # A(3,1)
        A[4:6, 0:2] = np.array([[t1,  1],
                                [-1, t1]])
        # A(2,3)
        A[2:4, 4:6] = np.array([[t2,  1],
                                [-1, t2]])
        # A(3,2)
        A[4:6, 2:4] = np.array([[t2, -1],
                                [ 1, t2]])
        # ===============================
        # 3) Convertir en Laplaciana
        # ===============================
        A = A - np.diag(np.sum(A, axis=1))
        # ===============================
        # 4) Autovalor mínimo
        # ===============================
        eigvals = np.linalg.eigvals(A)
        eigvals_sorted = np.sort(np.real(eigvals))
        k = eigvals_sorted[0]

        return A, k

    def build_state_matrix(self):
        names = sorted(self.herder_positions.keys())
        qmat = []
        for name in names:
            pose = self.herder_positions[name]
            qmat.append([
                pose.pose.position.x,
                pose.pose.position.y
            ])

        return np.array(qmat), names

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2*np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = AffineHerdingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
