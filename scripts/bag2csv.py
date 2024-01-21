# -*- coding: utf-8 -*-
# https://github.com/ros2/rosbag2/issues/473

import sqlite3
import csv
import sys
import yaml
import os
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]


if __name__ == "__main__":
        bag_file = sys.argv[1]+'/'+sys.argv[1]+'_0.db3'
        parser = BagFileParser(bag_file)
        aux = bag_file.split('/')
        aux.pop()
        metadata_file = ''.join(aux)+'/metadata.yaml'
        topic_list = []
        with open(metadata_file, 'r') as file:
            topics_file = yaml.load(file, Loader=yaml.FullLoader)
            topics_info = topics_file.get("rosbag2_bagfile_information").get("topics_with_message_count")
            for i in range(len(topics_info)):
                topic_aux = topics_info[i]['topic_metadata'].get("name")
                name = topics_info[i]['topic_metadata'].get("name")
                type = topics_info[i]['topic_metadata'].get("type")
                dir_file = sys.argv[1]+'/'+name[1:].replace('/', '-')+'.csv'
                
                # [position][0: time; 1: data]
                if type == "geometry_msgs/msg/Pose":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0], 
                                data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                                data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                        data_csv.append(aux)

                if type == "geometry_msgs/msg/PoseStamped":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0], 
                                data[i][1].pose.position.x, data[i][1].pose.position.y, data[i][1].pose.position.z,
                                data[i][1].pose.orientation.x, data[i][1].pose.orientation.y, data[i][1].pose.orientation.z, data[i][1].pose.orientation.w]
                        data_csv.append(aux)

                elif type == "std_msgs/msg/Float64" or type == "std_msgs/msg/Float32" or type == "std_msgs/msg/String":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'Data']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0], data[i][1].data]
                        data_csv.append(aux)

                elif type == "geometry_msgs/msg/Twist":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'VX', 'VY', 'VZ', 'WX', 'WY', 'WZ']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0],
                                data[i][1].linear.x, data[i][1].linear.y, data[i][1].linear.z,
                                data[i][1].angular.x, data[i][1].angular.y, data[i][1].angular.z]
                        data_csv.append(aux)

                elif type == "geometry_msgs/msg/PointStamped":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'X', 'Y', 'Z']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0],
                                data[i][1].point.x, data[i][1].point.y, data[i][1].point.z]
                        data_csv.append(aux)
                
                elif type == "geometry_msgs/msg/Vector3":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'X', 'Y', 'Z']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0],
                                data[i][1].x, data[i][1].y, data[i][1].z]
                        data_csv.append(aux)
                

                elif name == "/cpu_stats":
                    print('Converting topic: '+name+' ...')
                    data = parser.get_messages(name)[:][:]
                    header = ['Timestamp', 'CPU_Count', 'CPU_Percent', 'MEM_Percent', 'LoadAvg1', 'LoadAvg5', 'LoadAvg15']
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0],
                                data[i][1].data[0], data[i][1].data[1], data[i][1].data[2],
                                data[i][1].data[3], data[i][1].data[4], data[i][1].data[5]]
                        data_csv.append(aux)
                elif type == "rosgraph_msgs/msg/Clock":
                    print('Converting topic: '+name+' ...')
                    header = ['Timestamp', 'Sec', 'NanoSec']
                    data = parser.get_messages(name)[:][:]
                    data_csv = []
                    for i in range(len(parser.get_messages(name))):
                        aux = [ data[i][0],
                                data[i][1].clock.sec, data[i][1].clock.nanosec]
                        data_csv.append(aux)
                else:
                    # print('Undefined type: ' + type)
                    # print('Unconverted topic: ' + name)
                    continue
                try:
                    aux = bag_file.split('/')
                    aux.pop()
                    dir = ''.join(aux)+'/'+name[1:].replace('/', '-')+'.csv'
                    with open(dir, 'w', encoding='UTF8', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(header)
                        del header
                        writer.writerows(data_csv)
                        del data_csv
                except:
                    pass
