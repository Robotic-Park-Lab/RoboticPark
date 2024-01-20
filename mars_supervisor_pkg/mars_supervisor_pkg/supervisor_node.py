import logging
import time
import os
import rclpy
import yaml
import tf_transformations
from threading import Timer
import numpy as np
import random
from math import atan2, cos, sin, sqrt, pi

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        # Params
        self.declare_parameter('file', 'path')

        # Publisher
        self.publisher_list = []
        self.publisher_id = []
        
        # Subscription
        self.subscription_list = []
        self.subscription_id = []

        self.initialize()

    def initialize(self):
        self.get_logger().info('Supervisor::inicialize() ok.')

        cmd_file = self.get_parameter('file').get_parameter_value().string_value

        with open(cmd_file, 'r') as file:
                self.cmd = yaml.safe_load(file)
        
        
        for topic in self.cmd['config']['publisher'].keys():
            self.get_logger().info('Supervisor::Publisher: topic: %s' % (topic))
            if self.cmd['config']['publisher'][topic]['type'] == 'String':
                publisher = self.create_publisher(String,self.cmd['config']['publisher'][topic]['name'], 10)
                self.publisher_list.append(publisher)
                self.publisher_id.append(self.cmd['config']['publisher'][topic]['name'])
        
        for topic in self.cmd['config']['subscription'].keys():
            self.get_logger().info('Supervisor::Subscription: topic: %s' % (topic))
            if self.cmd['config']['subscription'][topic]['type'] == 'String':
                subscription = self.create_subscription(String, self.cmd['config']['subscription'][topic]['name'], self.string_callback, 10)
                self.subscription_list.append(subscription)
                self.subscription_id.append(self.cmd['config']['subscription'][topic]['name'])
    
        time = self.get_clock().now().to_msg()
        self.t_init = time.sec+time.nanosec*1e-9
        self.ready = False
        self.j = 0
        id = '00'
        self.checkout = 'null'
        if self.cmd['cmd'+id]['trigger']['type'] == 'time':
            self.t_ready = Timer(self.cmd['cmd'+id]['trigger']['value'], self._ready)
            self.get_logger().info('Supervisor::T: %f' % self.cmd['cmd'+id]['trigger']['value'])
            self.t_ready.start()
        else:
            self.checkout = self.cmd['cmd'+id]['trigger']['value']
            self.get_logger().info('Supervisor::Checkout: %s' % self.checkout)

        self.timer_task = self.create_timer(0.1, self.iterate)

        self.get_logger().info('Supervisor::inicialized.')

    def iterate(self):
        if self.ready:
            self.ready = False
            if self.j<10:
                id = '0'+str(self.j)
            else:
                id = str(self.j)


            idx = self.publisher_id.index(self.cmd['cmd'+id]['topic'])
            if self.cmd['cmd'+id]['type'] == 'Float64':
                msg = Float64()
                self.get_logger().info('Cmd%s: Value: %.3f' % (id, self.cmd['cmd'+id]['value']))
            elif self.cmd['cmd'+id]['type'] == 'String':
                msg = String()
                self.get_logger().info('Cmd%s: Value: %s' % (id, self.cmd['cmd'+id]['value']))
                if self.cmd['cmd'+id]['value'] == 'end':
                    self.destroy_node()
            
            msg.data = self.cmd['cmd'+id]['value']
            self.j = ((self.j+1) % len(self.cmd))
            self.publisher_list[idx].publish(msg)
            if self.j<10:
                id = '0'+str(self.j)
            else:
                id = str(self.j)
            if self.cmd['cmd'+id]['trigger']['type'] == 'time':
                self.t_ready = Timer(self.cmd['cmd'+id]['trigger']['value'], self._ready)
                self.get_logger().info('Supervisor::T: %f' % self.cmd['cmd'+id]['trigger']['value'])
                self.t_ready.start()
            else:
                self.checkout = self.cmd['cmd'+id]['trigger']['value']
                self.get_logger().info('Supervisor::Checkout: %s' % self.checkout)

    def _ready(self):
        self.ready = True
        self.get_logger().info('Supervisor::Ready:')
    
    def string_callback(self, msg):
        self.get_logger().info('Supervisor::Order: %s and Checkout: %s' % (msg.data, self.checkout))
        if msg.data == self.checkout:
            self.ready = True


def main(args=None):
    rclpy.init(args=args)
    supervisor_node = Supervisor()
    rclpy.spin(supervisor_node)

    supervisor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()