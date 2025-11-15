import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt16MultiArray, Float64, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose, Twist, Point, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from math import cos, sin, degrees, radians, pi, sqrt, asin, acos, atan2
from nav_msgs.msg import Path
from scipy.optimize import minimize

class Line():
    def __init__(self, point, vector):
        self.p = point
        self.v = vector

    def distance(self, point):
        pass

    def projection(self, point):
        pass


class Curve():
    def __init__(self, point, vector):
        self.p = point
        self.v = vector

    def distance(self, point):
        pass

    def projection(self, point):
        pass


class Sphere():
    def __init__(self, R = None, origin = None):
        self.R = R
        self.origin = origin

    def distance(self, point):
        error = point - self.origin
        distance2 = pow(error.x,2)+pow(error.y,2)+pow(error.z,2)
        distance = sqrt(distance2)-self.R
    
        return distance
    
    def value(self, point):
        z = point.z-self.origin.z
        value = pow(point.x,2)+pow(point.y,2)+pow(z,2)-self.R
        return value

    def projection(self, point):
        sphere_point = p = error = Point()
        error.x = point.x - self.origin.x
        error.y = point.y - self.origin.y
        error.z = point.z - self.origin.z
        distance2 = pow(error.x,2)+pow(error.y,2)+pow(error.z,2)
        distance = sqrt(distance2)
        p.x = (error.x/distance)*self.R+self.origin.x
        p.y = (error.y/distance)*self.R+self.origin.y
        p.z = (error.z/distance)*self.R+self.origin.z
        d1 = sqrt(pow(point.x-p.x,2)+pow(point.y-p.y,2)+pow(point.z-p.z,2))
        d2 = sqrt(pow(point.x+p.x,2)+pow(point.y+p.y,2)+pow(point.z+p.z,2))
        if d1<d2:
            c = 1.0
        else:
            c = -1.0
        sphere_point.x = c*p.x
        sphere_point.y = c*p.y
        sphere_point.z = c*p.z

        return sphere_point


class Cone():
    def __init__(self, a, c, origin = None):
        self.a = a
        self.c = c
        self.origin = origin

    def value(self, point):
        z = point.z-self.origin.z
        value = pow(point.x,2)/pow(self.a,2)+pow(point.y,2)/pow(self.a,2)-pow(z-self.c,2)/pow(self.c,2)
        return value

    def distance(self, point):
        pass

    def cone_constraint(self, vars):
        """Ecuación del cono (c = altura, a = radio)"""
        x, y, z = vars
        # Ecuación de la superficie del cono con vértice en origin.z + c
        return (x - self.origin.x)**2 / self.a**2 + (y - self.origin.y)**2 / self.a**2 - ((z - (self.origin.z + self.c))**2 / self.c**2)

    def projection(self, point):
        """Proyecta el punto sobre la superficie del cono mediante minimización numérica"""
        # Función objetivo: distancia al cuadrado
        def objective(vars):
            x, y, z = vars
            return (x - point.x)**2 + (y - point.y)**2 + (z - point.z)**2

        # Restricción: punto debe estar sobre el cono
        cons = ({
            'type': 'eq',
            'fun': self.cone_constraint
        })

        # Punto inicial: el propio punto
        x0 = [point.x, point.y, point.z]

        # Minimización con restricción
        res = minimize(objective, x0, constraints=cons, method='SLSQP')

        # Resultado
        proj = Point()
        proj.x, proj.y, proj.z = res.x
        return proj
    
    def projection_test(self, point):
        z = point.z-self.origin.z
        var_a = pow(point.x,2)+pow(point.y,2)
        var_b = pow(self.c,2)*pow(point.z-self.c,2)

        lambda1 = (sqrt(var_b)*pow(self.a,2)-sqrt(var_a)*pow(self.c,2))/(sqrt(var_a)+sqrt(var_b))
        lambda2 = (sqrt(var_b)*pow(self.a,2)-sqrt(var_a)*pow(self.c,2))/(sqrt(var_a)-sqrt(var_b))

        npoint1 = npoint2 = Point()
        npoint1.x = (pow(self.a,2)*point.x)/(pow(self.a,2)-lambda1)
        npoint1.y = (pow(self.a,2)*point.y)/(pow(self.a,2)-lambda1)
        npoint1.z = (pow(self.c,2)*(z+lambda1/self.c))/(pow(self.c,2)+lambda1)

        npoint2.x = (pow(self.a,2)*point.x)/(pow(self.a,2)-lambda2)
        npoint2.y = (pow(self.a,2)*point.y)/(pow(self.a,2)-lambda2)
        npoint2.z = (pow(self.c,2)*(z+lambda1/self.c))/(pow(self.c,2)+lambda2)

        d1 = sqrt(pow(point.x-npoint1.x,2)+pow(point.y-npoint1.y,2)+pow(z-npoint1.z,2))
        d2 = sqrt(pow(point.x-npoint2.x,2)+pow(point.y-npoint2.y,2)+pow(z-npoint2.z,2))
        if d1<d2:
            npoint1.z = npoint1.z+self.origin.z
            return npoint1
        else:
            npoint2.z = npoint2.z+self.origin.z
            return npoint2
        '''
        mod = sqrt(pow(point.x-self.origin.x,2)+pow(point.y-self.origin.y,2))
        dir_x = (point.x-self.origin.x)/mod
        dir_y = (point.y-self.origin.y)/mod
        alpha = atan2(self.a,self.c)
        alpha = 0.4298
        gx = dir_x*cos(alpha)
        gy = dir_y*cos(alpha)
        gz = -sin(alpha)

        mod = pow(gx,2)+pow(gy,2)+pow(gz,2)
        lambda1 = ((point.x-self.origin.x)*gx+(point.y-self.origin.y)*gy+(point.z-(self.c+self.origin.z))*gz)/mod
        npoint1 = Point()
        npoint1.x = self.origin.x + lambda1*gx
        npoint1.y = self.origin.y + lambda1*gy
        npoint1.z = self.c+self.origin.z + lambda1*gz

        return npoint1
        '''

class Ellipsoid():
    def __init__(self, a = None, b = None, c = None, origin = None):
        self.a = a
        self.b = b
        self.c = c
        self.origin = origin

    def value(self, point):
        z = point.z-self.origin.z
        value = pow(point.x,2)/pow(self.a,2)+pow(point.y,2)/pow(self.b,2)+pow(z,2)/pow(self.c,2)-1
        
        return value

class LagrangeMultipliers(Node):
    def __init__(self):
        super().__init__('lagrange_multipliers')
        # Params
        self.declare_parameter('example', 'value')

        # Publisher
        self.publisher_example_ = self.create_publisher(Float64,'/example_float', 10)
        
        # Subscription
        self.sub_example = self.create_subscription(String, '/example_string', self.example_callback, 10)
        
        self.initialize()
    
    def initialize(self):
        self.get_logger().info('LagrangeMultipliers::inicialize() ok.')

    def example_callback(self, msg):
        data = msg
        self.get_logger().info('New msg: %s' % (data))

def main(args=None):
    rclpy.init(args=args)
    lagrange_node = LagrangeMultipliers()
    rclpy.spin(lagrange_node)

    lagrange_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
