#!/usr/bin/python3
import sys
import os
import subprocess
import time

import yaml

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose
from visualization_msgs.msg import *
from ExperimentDesign import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

qtCreatorFile = "ExperimentDesign.ui" 
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)

class Node:
    def __init__(self, id, executable, name, package, file):
        self.id = id
        self.executable = executable
        self.name = name
        self.package = package
        self.file = file
    
    def dictionary_output(self):
        aux = {'node':{'executable':self.executable,'name':self.name,'pkg':self.package,'file':self.file}}
        return aux

class Robot:
    def __init__(self, id, model, tipo, name, control_mode, positioning, pose, yaw, uri, controller_type, controller_period, controller_co, controller_ai, communication_type, communication_path, communication_period, communication_co, task_enable, task_onboard, task_controller, task_period, task_co, task_ai, task_gain, task_role, task_type, task_relationship):
        self.id = id
        self.model = model
        self.type = tipo
        self.name = name
        self.control_mode = control_mode
        self.positioning = positioning
        self.pose = pose
        self.yaw = yaw
        self.uri = uri
        self.controller_type = controller_type
        self.controller_period = controller_period
        self.controller_co = controller_co
        self.controller_ai = controller_ai
        self.communication_type = communication_type
        self.communication_path = communication_path
        self.communication_period = communication_period
        self.communication_co = communication_co
        self.task_enable = task_enable
        self.task_onboard = task_onboard
        self.task_controller = task_controller
        self.task_period = task_period
        self.task_co = task_co
        self.task_ai = task_ai
        self.task_gain = task_gain
        self.task_role = task_role
        self.task_type = task_type
        self.task_relationship = task_relationship

    def dictionary_output(self):
        aux = {'model': self.model, 'type':self.type,'name':self.name,'control_mode':self.control_mode,'positioning':self.positioning,'pose': self.pose, 'yaw': self.yaw,
                                 'uri': self.uri,'controller':{'type':self.controller_type,'enable':'True','protocol':'Continuous',
                                 'period':self.controller_period,'threshold':{'type':'Constant','co':self.controller_co,'ai':self.controller_ai}},'communication':{'type':self.communication_type,
                                 'threshold':{'type':'Constant','co':self.communication_co,'ai':0.0}},'local_pose':{'enable':'True','path':self.communication_path,'T':self.communication_period },
                                 'task':{'enable':self.task_enable,'T':self.task_period,'Onboard':self.task_onboard,'controller':{'type':self.task_controller,'protocol':'Continuous','period':self.task_period,'upperLimit':0.1,
                                 'lowerLimit':-0.1,'gain':self.task_gain,'threshold':{'type':'Constant','co':self.task_co,'ai':self.task_ai}},'role':self.task_role,'type':self.task_type,'relationship':self.task_relationship}}
        return aux

class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
           
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.NameFile_Button.clicked.connect(self.SaveFiles)
        #
        self.processes = ''
        self.model_cpu = QtGui.QStandardItemModel()
        self.Monitoring_List.setModel(self.model_cpu)
        self.Monitoring_Add_Button.clicked.connect(self.AddTopic2Monitoring)
        self.Monitoring_Remove_Button.clicked.connect(self.RemoveTopic2Monitoring)
        #
        self.topics = ''
        self.model_data = QtGui.QStandardItemModel()
        self.DataLogging_Edit_List.setModel(self.model_data)
        self.DataLogging_Add_button.clicked.connect(self.AddTopic2DataLogging)
        self.DataLogging_Remove_button.clicked.connect(self.RemoveTopic2DataLogging)
        #
        self.arch_nodes = []
        self.Architecture_Add_button.clicked.connect(self.AddNode2Architecture)
        self.Architecture_Remove_button.clicked.connect(self.RemoveNode2Architecture)
        # 
        self.agents_nodes = []
        self.Agent_Add_button.clicked.connect(self.AddNode2Agent)
        self.Agent_Remove_button.clicked.connect(self.RemoveNode2Agent)
        #
        self.robot_list = []
        self.relationship = ''
        self.model_relationship = QtGui.QStandardItemModel()
        self.Relationship_list.setModel(self.model_relationship)
        self.Relationship_Add_Buttom.clicked.connect(self.AddRelationship)
        self.Relationship_Remove_Buttom.clicked.connect(self.RemoveRelationship)
        self.Add_Button.clicked.connect(self.AddRobot)
        self.Remove_Button.clicked.connect(self.RemoveRobot)
        #
        self.actionImport_2.triggered.connect(self.importData)

        self.dictionary = {
            'Operation': {'mode':'virtual', 'tool':'Null','world':'Null'},
            'Experience': {'type':'formation'},
            'Architecture': {'mode':'centralized'},
            'CPU_Monitoring':{'enable': 'True', 'node':{'executable':'measure_process','name':'benchmark','pkg':'measure_process_ros2_pkg'},'processes':'ros2'},
            'Interface':{'enable': 'False', 'rviz2':{'enable':'False', 'node':{'executable':'rviz2','name':'rviz2','pkg':'rviz2'},'file':'rviz_config_path'},
                                            'rqt':{'enable':'True', 'node':{'executable':'rqt_gui','name':'rqt','pkg':'rqt_gui'},'file':'rqt_config_path'},
                                            'own':{'enable':'False', 'node':{'executable':'own','name':'own','pkg':'own'},'file':'own_config_path'}},
            'Data_Logging':{'enable': 'False', 'all':'False', 'name':'date','topics':'Null'},
            'Supervisor':{'enable': 'False', 'node':{'executable':'','name':'','pkg':'', 'file':'config_path'}},
            'Other':{},
            'Robots':{}
        }
        
    def importData(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Open File",r"../../roboticpark_config/resources","Config Files (*.yaml *.yml)")
        with open(fileName, 'r') as file:
            documents = yaml.safe_load(file)
        # Operation mode
        if documents['Operation']['mode'] == 'physical':
            self.OperationMode_ComboBox.setCurrentIndex(0)
        elif documents['Operation']['mode'] == 'virtual':
            self.OperationMode_ComboBox.setCurrentIndex(1)
        else:
            self.OperationMode_ComboBox.setCurrentIndex(2)
        # Operation tool
        if documents['Operation']['tool'] == 'Webots':
            self.OperationMode_Tool_ComboBox.setCurrentIndex(0)
        elif documents['Operation']['tool'] == 'Gazebo':
            self.OperationMode_Tool_ComboBox.setCurrentIndex(1)
        else:
            self.OperationMode_Tool_ComboBox.setCurrentIndex(2)
        # Operation world
        self.OperationMode_World_edit.setText(documents['Operation']['world'])
        # Experience
        if documents['Experience']['type'] == 'formation':
            self.Experience_combo.setCurrentIndex(0)
        elif documents['Experience']['type'] == 'identification':
            self.Experience_combo.setCurrentIndex(1)
        else:
            self.Experience_combo.setCurrentIndex(2)
        # Monitoring
        if documents['CPU_Monitoring']['enable']:
            self.MonitoringEnable_Check.setChecked(True) 
            self.processes = documents['CPU_Monitoring']['processes']
            aux0 = self.processes.split(', ')
            for topic in aux0:
                item = QStandardItem(topic)
                self.model_cpu.appendRow(item)
        # Data Logging
        if documents['Data_Logging']['enable']:
            self.DataLogging_Enable_check.setChecked(True) 
            if documents['Data_Logging']['all']:
                self.DataLogging_All_check.setChecked(True)
            
            self.DataLogging_Name_edit.setText(documents['Data_Logging']['name'])
            self.topics = documents['Data_Logging']['topics']
            aux0 = self.topics.split(' ')
            for topic in aux0:
                item = QStandardItem(topic)
                self.model_data.appendRow(item)
        # Architecture
        if documents['Architecture']['mode'] == 'Centralized':
            self.Architecture_ComboBox.setCurrentIndex(0)
        elif documents['Architecture']['mode'] == 'Distributed':
            self.Architecture_ComboBox.setCurrentIndex(1)
        else:
            self.Architecture_ComboBox.setCurrentIndex(2)
        for node in list(documents['Architecture'].keys()):
            if not node == 'mode':
                rowPosition = self.Architecture_NodesList.rowCount()
                self.Architecture_NodesList.insertRow(rowPosition)
                self.Architecture_NodesList.setItem(rowPosition , 0, QTableWidgetItem(node))
                self.Architecture_NodesList.setItem(rowPosition , 1, QTableWidgetItem(documents['Architecture'][node]['executable']))
                self.Architecture_NodesList.setItem(rowPosition , 2, QTableWidgetItem(documents['Architecture'][node]['name']))
                self.Architecture_NodesList.setItem(rowPosition , 3, QTableWidgetItem(documents['Architecture'][node]['pkg']))
                self.Architecture_NodesList.setItem(rowPosition , 4, QTableWidgetItem(documents['Architecture'][node]['file']))
                new_node = Node(node, documents['Architecture'][node]['executable'], documents['Architecture'][node]['name'], documents['Architecture'][node]['pkg'], documents['Architecture'][node]['file'])
                self.arch_nodes.append(new_node)
        
        # Interface
        if documents['Interface']['enable']:
            self.Interface_Enable_check.setChecked(True)
            if documents['Interface']['rviz2']['enable']:
                self.Interface_RVIZ2_check.setChecked(True)
                self.Interface_RVIZ2_File_edit.setText(documents['Interface']['rviz2']['file'])
            if documents['Interface']['rqt']['enable']:
                self.Interface_RQT_check.setChecked(True)
                self.Interface_RQT_File_edit.setText(documents['Interface']['rqt']['file'])
        # Supervisor
        if documents['Supervisor']['enable']:
            self.Supervisor_check.setChecked(True)
            self.Supervisor_Executable_edit.setText(documents['Supervisor']['node']['executable'])
            self.Supervisor_Name_edit.setText(documents['Supervisor']['node']['name'])
            self.Supervisor_Package_edit.setText(documents['Supervisor']['node']['pkg'])
            self.Supervisor_File_edit.setText(documents['Supervisor']['node']['file'])
        # Other
        for node in list(documents['Other'].keys()):
            rowPosition = self.Agent_Table.rowCount()
            self.Agent_Table.insertRow(rowPosition)
            self.Agent_Table.setItem(rowPosition , 0, QTableWidgetItem(node))
            self.Agent_Table.setItem(rowPosition , 1, QTableWidgetItem(documents['Other'][node]['executable']))
            self.Agent_Table.setItem(rowPosition , 2, QTableWidgetItem(documents['Other'][node]['name']))
            self.Agent_Table.setItem(rowPosition , 3, QTableWidgetItem(documents['Other'][node]['pkg']))
            self.Agent_Table.setItem(rowPosition , 4, QTableWidgetItem(documents['Other'][node]['file']))
            new_node = Node(node, documents['Other'][node]['executable'], documents['Other'][node]['name'], documents['Other'][node]['pkg'], documents['Other'][node]['file'])
            self.agents_nodes.append(new_node)
        # Robots
        for robot in list(documents['Robots'].keys()):
            rowPosition = self.Robot_Table.rowCount()
            self.Robot_Table.insertRow(rowPosition)
            id = robot
            name = documents['Robots'][robot]['name']
            if "dron" in name:
                model = 'Crazyflie 2.1'
            else:
                model = 'Khepera IV'

            mode = documents['Robots'][robot]['type']
            control_mode = documents['Robots'][robot]['control_mode']
            positioning = documents['Robots'][robot]['positioning']
            pose = documents['Robots'][robot]['pose']
            yaw = '0.0'
            uri = documents['Robots'][robot]['uri']
            controller_type = documents['Robots'][robot]['controller']['type']
            controller_period = documents['Robots'][robot]['controller']['period']
            controller_co = documents['Robots'][robot]['controller']['threshold']['co']
            controller_ai = documents['Robots'][robot]['controller']['threshold']['ai']
            communication_type = documents['Robots'][robot]['communication']['type']
            communication_path = documents['Robots'][robot]['local_pose']['path']
            communication_period = documents['Robots'][robot]['local_pose']['T']
            communication_co = documents['Robots'][robot]['communication']['threshold']['co']
            task_enable = documents['Robots'][robot]['task']['enable']
            task_onboard = documents['Robots'][robot]['task']['Onboard']
            task_controller = documents['Robots'][robot]['task']['controller']['type']
            task_period = documents['Robots'][robot]['task']['controller']['period']
            task_co = documents['Robots'][robot]['task']['controller']['threshold']['co']
            task_ai = documents['Robots'][robot]['task']['controller']['threshold']['ai']
            task_gain = documents['Robots'][robot]['task']['controller']['gain']
            task_role = documents['Robots'][robot]['task']['role']
            task_type = documents['Robots'][robot]['task']['type']
            task_relationship = documents['Robots'][robot]['task']['relationship']
            new_robot = Robot(id, model, mode, name, control_mode, positioning, pose, yaw, uri, controller_type, controller_period, controller_co, controller_ai, communication_type, communication_path, communication_period, communication_co, task_enable, task_onboard, task_controller, task_period, task_co, task_ai, task_gain, task_role, task_type, task_relationship)
            self.robot_list.append(new_robot)

            self.Robot_Table.setItem(rowPosition , 0, QTableWidgetItem(id))
            self.Robot_Table.setItem(rowPosition , 1, QTableWidgetItem(positioning))
            self.Robot_Table.setItem(rowPosition , 2, QTableWidgetItem(controller_type))
            self.Robot_Table.setItem(rowPosition , 3, QTableWidgetItem(communication_type))
            self.Robot_Table.setItem(rowPosition , 4, QTableWidgetItem(str(task_enable)))
            self.Robot_Table.setItem(rowPosition , 5, QTableWidgetItem(task_controller))
            self.Robot_Table.setItem(rowPosition , 6, QTableWidgetItem(task_type))
            self.Robot_Table.setItem(rowPosition , 7, QTableWidgetItem(task_relationship))
            self.relationship = ''
            self.model_relationship.removeRows( 0, self.model_relationship.rowCount())

    def AddRobot(self):
        rowPosition = self.Robot_Table.rowCount()
        self.Robot_Table.insertRow(rowPosition)
        if rowPosition<10:
            id = 'Robot0'+str(rowPosition)
            
        else:
            id = 'Robot'+str(rowPosition)
        model = self.TypeRobot_combo.currentText()
        mode = self.comboBox.currentText()
        name = self.IdRobot_text.text()
        control_mode = 'HighLevel'
        positioning = self.Positioning_Combo.currentText()
        pose = self.Robot_X_text.text()+' '+self.Robot_Y_text.text()
        yaw = self.Robot_Yaw_edit.text()
        uri = self.RobotURI_edit.text()
        controller_type = self.InternalController_Combo.currentText()
        controller_period = self.InternalController_Period_label.text()
        controller_co = self.InternalController_Co_edit.text()
        controller_ai = self.InternalController_ai_edit.text()
        communication_type = self.Communication_Combo.currentText()
        communication_path = self.TaskPath_check.isChecked()
        communication_path = True
        communication_period = self.Communication_Period_edit.text()
        communication_co = self.Communication_Co_edit.text()
        task_enable = self.TaskEnable_check.isChecked()
        task_onboard = self.TaskOnboard_check.isChecked()
        task_controller = self.TaskController_Combo.currentText()
        task_period = self.TaskPeriod_edit.text()
        task_co = self.Task_co_edit.text()
        task_ai = self.Task_ai_edit.text()
        task_gain = self.TaskGain_edit.text()
        task_role = self.Role_Combo.currentText()
        task_type = self.TaskType_combo.currentText()
        task_relationship = self.relationship
        new_robot = Robot(id, model, mode, name, control_mode, positioning, pose, yaw, uri, controller_type, controller_period, controller_co, controller_ai, communication_type, communication_path, communication_period, communication_co, task_enable, task_onboard, task_controller, task_period, task_co, task_ai, task_gain, task_role, task_type, task_relationship)
        self.robot_list.append(new_robot)

        self.Robot_Table.setItem(rowPosition , 0, QTableWidgetItem(id))
        self.Robot_Table.setItem(rowPosition , 1, QTableWidgetItem(positioning))
        self.Robot_Table.setItem(rowPosition , 2, QTableWidgetItem(controller_type))
        self.Robot_Table.setItem(rowPosition , 3, QTableWidgetItem(communication_type))
        self.Robot_Table.setItem(rowPosition , 4, QTableWidgetItem(str(task_enable)))
        self.Robot_Table.setItem(rowPosition , 5, QTableWidgetItem(task_controller))
        self.Robot_Table.setItem(rowPosition , 6, QTableWidgetItem(task_type))
        self.Robot_Table.setItem(rowPosition , 7, QTableWidgetItem(task_relationship))
        self.relationship = ''
        self.model_relationship.removeRows( 0, self.model_relationship.rowCount())

    def RemoveRobot(self):
        for i in range(len(self.robot_list)):
            if self.robot_list[i].name == self.IdRobot_text.text():
                self.Robot_Table.removeRow(i)
                self.robot_list.pop(i)
                break

    def AddRelationship(self):
        agent = self.TaskRelationship_agent_edit.text()
        value= self.TaskRelationship_value_edit.text()
        string = agent+': '+value
        item = QStandardItem(string)
        self.model_relationship.appendRow(item)
        if self.relationship == '':
            self.relationship = agent+'_'+value
        else:
            self.relationship = self.relationship +', '+agent+'_'+value
        
    def RemoveRelationship(self):
        agent = self.TaskRelationship_agent_edit.text()
        value= self.TaskRelationship_value_edit.text()
        items = self.model_relationship.findItems(agent+': '+value, QtCore.Qt.MatchStartsWith)
        if len(items) > 0:
            for item in items:
                if agent+': '+value:
                    self.model_relationship.takeRow(item.row()) #take row of item
        else:
            print('not found')
        self.relationship = self.relationship.replace(agent+'_'+value+', ','')
        self.relationship = self.relationship.replace(', '+agent+'_'+value,'')
        self.relationship = self.relationship.replace(agent+'_'+value,'')

    def AddTopic2Monitoring(self):
        topic = self.Monitoring_Process_Edit.text()
        item = QStandardItem(topic)
        self.model_cpu.appendRow(item)
        if self.processes == '':
            self.processes = topic
        else:
            self.processes = self.processes +', '+topic
        

    def RemoveTopic2Monitoring(self):
        topic = self.Monitoring_Process_Edit.text()
        items = self.model_cpu.findItems(topic, QtCore.Qt.MatchStartsWith)
        if len(items) > 0:
            for item in items:
                if topic:
                    self.model_cpu.takeRow(item.row()) #take row of item
        else:
            print('not found')
        self.processes = self.processes.replace(topic+', ','')
        self.processes = self.processes.replace(', '+topic,'')
        self.processes = self.processes.replace(topic,'')

    def AddTopic2DataLogging(self):
        topic = self.DataLogging_Topics_edit.text()
        item = QStandardItem(topic)
        self.model_data.appendRow(item)
        if self.topics == '':
            self.topics = topic
        else:
            self.topics = self.topics +' '+topic
        

    def RemoveTopic2DataLogging(self):
        topic = self.DataLogging_Topics_edit.text()
        items = self.model_data.findItems(topic, QtCore.Qt.MatchStartsWith)
        if len(items) > 0:
            for item in items:
                if topic:
                    self.model_data.takeRow(item.row()) #take row of item
        else:
            print('not found')
        self.topics = self.topics.replace(topic+', ','')
        self.topics = self.topics.replace(', '+topic,'')
        self.topics = self.topics.replace(topic,'')

    def AddNode2Architecture(self):
        rowPosition = self.Architecture_NodesList.rowCount()
        self.Architecture_NodesList.insertRow(rowPosition)
        if rowPosition<10:
            id = 'node0'+str(rowPosition)
            
        else:
            id = 'node'+str(rowPosition)
        self.Architecture_NodesList.setItem(rowPosition , 0, QTableWidgetItem(id))
        self.Architecture_NodesList.setItem(rowPosition , 1, QTableWidgetItem(self.Architecture_NodeExec_edit.text()))
        self.Architecture_NodesList.setItem(rowPosition , 2, QTableWidgetItem(self.Architecture_NodeName_edit.text()))
        self.Architecture_NodesList.setItem(rowPosition , 3, QTableWidgetItem(self.Architecture_NodePkg_edit.text()))
        self.Architecture_NodesList.setItem(rowPosition , 4, QTableWidgetItem(self.Architecture_File_edit.text()))
        node = Node(id, self.Architecture_NodeExec_edit.text(), self.Architecture_NodeName_edit.text(), self.Architecture_NodePkg_edit.text(), self.Architecture_File_edit.text())
        self.arch_nodes.append(node)

    def RemoveNode2Architecture(self):
        for i in range(len(self.arch_nodes)):
            if self.arch_nodes[i].name == self.Architecture_NodeName_edit.text():
                self.Architecture_NodesList.removeRow(i)
                self.arch_nodes.pop(i)
                break
    
    def AddNode2Agent(self):
        rowPosition = self.Agent_Table.rowCount()
        self.Agent_Table.insertRow(rowPosition)
        self.Agent_Table.setItem(rowPosition , 0, QTableWidgetItem(self.Agent_Node_edit.text()))
        self.Agent_Table.setItem(rowPosition , 1, QTableWidgetItem(self.Agent_Executable_edit.text()))
        self.Agent_Table.setItem(rowPosition , 2, QTableWidgetItem(self.Agent_Name_edit.text()))
        self.Agent_Table.setItem(rowPosition , 3, QTableWidgetItem(self.Agent_Package_edit.text()))
        self.Agent_Table.setItem(rowPosition , 4, QTableWidgetItem(self.Agent_File_edit.text()))
        node = Node(self.Agent_Node_edit.text(), self.Agent_Executable_edit.text(), self.Agent_Name_edit.text(), self.Agent_Package_edit.text(), self.Agent_File_edit.text())
        self.agents_nodes.append(node)

    def RemoveNode2Agent(self):
        for i in range(len(self.agents_nodes)):
            if self.agents_nodes[i].id == self.Agent_Node_edit.text():
                self.Agent_Table.removeRow(i)
                self.agents_nodes.pop(i)
                break

    def SaveFiles(self):
        file_name = self.NameFile_Edit.text()
        if file_name == '':
            print('Invalid file name')
        else:
            # Robots
            if not len(self.robot_list) == 0:
                for robot in self.robot_list:
                    self.dictionary['Robots'][robot.id] = robot.dictionary_output()

            # Operation mode
            self.dictionary['Operation']['mode'] = self.OperationMode_ComboBox.currentText()
            if not self.dictionary['Operation']['mode'] == 'physical':
                self.dictionary['Operation']['tool'] = self.OperationMode_Tool_ComboBox.currentText()
                if self.dictionary['Operation']['tool'] == 'Webots':
                    if self.OperationMode_World_edit.text() == '':
                        self.BuildWebotsWorld(file_name)
                        self.dictionary['Operation']['world'] = file_name+'.wbt'
                    else:
                        self.dictionary['Operation']['world'] = self.OperationMode_World_edit.text()
                else:
                    if self.OperationMode_World_edit.text() == '':
                        self.BuildGazeboWorld(file_name)
                        self.dictionary['Operation']['world'] = file_name+'.world'
                    else:
                        self.dictionary['Operation']['world'] = self.OperationMode_World_edit.text()
            
            # Experience
            self.dictionary['Experience']['type'] = self.Experience_combo.currentText()

            # CPU Monitoring
            if self.MonitoringEnable_Check.isChecked():
                self.dictionary['CPU_Monitoring']['enable'] = 'True'
                self.dictionary['CPU_Monitoring']['processes'] = self.processes

            # Architecture
            self.dictionary['Architecture']['mode'] = self.Architecture_ComboBox.currentText()
            if self.dictionary['Architecture']['mode'] == 'Centralized' and len(self.arch_nodes) == 0:
                self.dictionary['Architecture']['node00'] = {'name':'name','executable':'centralized_formation_controller','name':'formation_controller','pkg':'uned_swarm_task','file':'config_path'}
            else:
                for node in self.arch_nodes:
                    self.dictionary['Architecture'][node.id] = node.dictionary_output()

            # Interfaces
            if self.Interface_Enable_check.isChecked():
                if self.Interface_RVIZ2_check.isChecked():
                    self.dictionary['Interface']['rviz2']['enable'] = 'True'
                    if self.Interface_RVIZ2_File_edit.text() == '':
                        self.BuildRVIZFile(file_name)
                        self.dictionary['Interface']['rviz2']['file'] = file_name+'.rviz'
                    else:
                        self.dictionary['Interface']['rviz2']['file'] = self.Interface_RVIZ2_File_edit.text()
                if self.Interface_RQT_check.isChecked():
                    self.dictionary['Interface']['rqt']['enable'] = 'True'
                    self.dictionary['Interface']['rqt']['file'] = self.Interface_RQT_File_edit.text()

            # Data Logging
            if self.DataLogging_Enable_check.isChecked():
                self.dictionary['Data_Logging']['enable'] = 'True'
                self.dictionary['Data_Logging']['name'] = self.DataLogging_Name_edit.text()
                if self.DataLogging_All_check.isChecked():
                    self.dictionary['Data_Logging']['all'] = 'True'
                else:
                    self.dictionary['Data_Logging']['topics'] = self.topics

            # Supervisor
            if self.Supervisor_check.isChecked():
                self.dictionary['Supervisor']['enable'] = 'True'
                self.dictionary['Supervisor']['node']['executable'] = self.Supervisor_Executable_edit.text()
                self.dictionary['Supervisor']['node']['name'] = self.Supervisor_Name_edit.text()
                self.dictionary['Supervisor']['node']['pkg'] = self.Supervisor_Package_edit.text()
                self.dictionary['Supervisor']['node']['file'] = self.Supervisor_File_edit.text()

            # Other Agents
            if not len(self.agents_nodes) == 0:
                for node in self.agents_nodes:
                    self.dictionary['Other'][node.id] = node.dictionary_output()

            

            yaml_file = open(file_name+'.yaml', 'w')
            yaml.dump(self.dictionary,yaml_file)
            print(file_name+'.yaml saved')
            yaml_file.close()

    def BuildWebotsWorld(self,name):
        f = open(name+'.wbt', "w")
        f.write("#VRML_SIM R2023b utf8")
        f.write("\n")
        f.write('\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"')
        f.write('\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/Floor.proto"')
        f.write('\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/appearances/protos/Parquetry.proto"')
        f.write('\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/k-team/khepera4/protos/Khepera4.proto"')
        f.write('\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto"')
        f.write('\n')
        f.write('\nWorldInfo {')
        f.write('\n    info [')
        f.write('\n        "Robotic Park"')
        f.write('\n    ]')
        f.write('\n    basicTimeStep 20')
        f.write('\n}')
        f.write('\nViewpoint {')
        f.write('\n    fieldOfView 1')
        f.write('\n    orientation 0.08257814004826021 0.7347446275807098 -0.6732987323821825 0.7219016991902246')
        f.write('\n    position -6.686523789804856 3.246953734178069 5.0217815008034')
        f.write('\n}')
        f.write('\nTexturedBackground {')
        f.write('\n}')
        f.write('\nDirectionalLight {')
        f.write('\n    ambientIntensity 1')
        f.write('\n    direction 6 -16 -10')
        f.write('\n    intensity 4')
        f.write('\n}')
        f.write('\nFloor {')
        f.write('\n    translation -1 0 0')
        f.write('\n    size 8 6')
        f.write('\n    appearance Parquetry {')
        f.write('\n        type "chequered"')
        f.write('\n        IBLStrength 0')
        f.write('\n    }')
        f.write('\n}')
        f.write('\nSolid {')
        f.write('\n    rotation 0 0 1 3.14159')
        f.write('\n    children [')
        f.write('\n        CadShape {')
        f.write('\n        url [')
        f.write('\n            "meshes/RoboticPark.dae"')
        f.write('\n        ]')
        f.write('\n        }')
        f.write('\n    ]')
        f.write('\n    contactMaterial ""')
        f.write('\n    boundingObject Mesh {')
        f.write('\n         url [')
        f.write('\n        "meshes/RoboticPark_new.dae"')
        f.write('\n        ]')
        f.write('\n    }')
        f.write('\n    radarCrossSection 1')
        f.write('\n}')
        f.write('\n')
        
        if not len(self.robot_list) == 0:
            for robot in self.robot_list:
                if not robot.type == 'physical':
                    if robot.model == 'Crazyflie 2.1':
                        f.write('\nCrazyflie {')
                        f.write('\n  translation '+robot.pose+' 0')
                        f.write('\n  name "'+robot.name+'"')
                        f.write('\n  controller "<extern>"')
                        f.write('\n  supervisor TRUE')
                        f.write('\n  extensionSlot [')
                        f.write('\n    InertialUnit {')
                        f.write('\n    }')
                        f.write('\n  ]')
                        f.write('\n}')
                        f.write('\n')
                    if robot.model == 'Khepera IV':
                        f.write('\Khepera4 {')
                        f.write('\n  translation '+robot.pose+' 0.015')
                        f.write('\n  rotation 0 0 1 '+robot.yaw)
                        f.write('\n  name "'+robot.name+'"')
                        f.write('\n  controller "<extern>"')
                        f.write('\n  supervisor TRUE')
                        f.write('\n  turretSlot [')
                        f.write('\n    GPS {')
                        f.write('\n    }')
                        f.write('\n    InertialUnit {')
                        f.write('\n    }')
                        f.write('\n  ]')
                        f.write('\n}')
                        f.write('\n')
        f.close()

    def BuildGazeboWorld(self,name):
        f = open(name+'.world', "w")

        f.close()

    def BuildRVIZFile(self,name):
        f = open(name+'.rviz', "w")
        f.write('\nPanels:')
        f.write('\n  - Class: rviz_common/Displays')
        f.write('\n    Help Height: 78')
        f.write('\n    Name: Displays')
        f.write('\n    Property Tree Widget:')
        f.write('\n      Expanded:')
        f.write('\n        - /Dron011/Status1')
        f.write('\n      Splitter Ratio: 0.5')
        f.write('\n    Tree Height: 549')
        f.write('\n  - Class: rviz_common/Selection')
        f.write('\n    Name: Selection')
        f.write('\n  - Class: rviz_common/Tool Properties')
        f.write('\n    Expanded:')
        f.write('\n      - /2D Goal Pose1')
        f.write('\n      - /Publish Point1')
        f.write('\n    Name: Tool Properties')
        f.write('\n    Splitter Ratio: 0.5886790156364441')
        f.write('\n  - Class: rviz_common/Views')
        f.write('\n    Expanded:')
        f.write('\n      - /Current View1')
        f.write('\n    Name: Views')
        f.write('\n    Splitter Ratio: 0.5')
        f.write('\n  - Class: rviz_common/Time')
        f.write('\n    Experimental: false')
        f.write('\n    Name: Time')
        f.write('\n    SyncMode: 0')
        f.write('\n    SyncSource: ""')
        f.write('\nVisualization Manager:')
        f.write('\n  Class: ""')
        f.write('\n  Displays:')
        f.write('\n    - Alpha: 0.5')
        f.write('\n      Cell Size: 1')
        f.write('\n      Class: rviz_default_plugins/Grid')
        f.write('\n      Color: 160; 160; 164')
        f.write('\n      Enabled: true')
        f.write('\n      Line Style:')
        f.write('\n        Line Width: 0.029999999329447746')
        f.write('\n        Value: Lines')
        f.write('\n      Name: Grid')
        f.write('\n      Normal Cell Count: 0')
        f.write('\n      Offset:')
        f.write('\n        X: 0')
        f.write('\n        Y: 0')
        f.write('\n        Z: 0')
        f.write('\n      Plane: XY')
        f.write('\n      Plane Cell Count: 10')
        f.write('\n      Reference Frame: <Fixed Frame>')
        f.write('\n      Value: true')
        f.write('\n    - Alpha: 1')
        f.write('\n      Class: rviz_default_plugins/RobotModel')
        f.write('\n      Collision Enabled: false')
        f.write('\n      Description File: /home/kiko/roboticpark_ws/src/uned_crazyflie_ros_pkg/uned_crazyflie_config/model/urdf/RoboticPark_simple.urdf')
        f.write('\n      Description Source: File')
        f.write('\n      Description Topic:')
        f.write('\n        Depth: 5')
        f.write('\n        Durability Policy: Volatile')
        f.write('\n        History Policy: Keep Last')
        f.write('\n        Reliability Policy: Reliable')
        f.write('\n        Value: ""')
        f.write('\n      Enabled: true')
        f.write('\n      Links:')
        f.write('\n        All Links Enabled: true')
        f.write('\n        Expand Joint Details: false')
        f.write('\n        Expand Link Details: false')
        f.write('\n        Expand Tree: false')
        f.write('\n        Link Tree Style: Links in Alphabetic Order')
        f.write('\n        base_link:')
        f.write('\n          Alpha: 1')
        f.write('\n          Show Axes: false')
        f.write('\n          Show Trail: false')
        f.write('\n          Value: true')
        f.write('\n      Mass Properties:')
        f.write('\n        Inertia: false')
        f.write('\n        Mass: false')
        f.write('\n      Name: RobotModel')
        f.write('\n      TF Prefix: RoboticPark')
        f.write('\n      Update Interval: 0')
        f.write('\n      Value: true')
        f.write('\n      Visual Enabled: true')
        if not len(self.robot_list) == 0:
            for robot in self.robot_list:
                f.write('\n    - Alpha: 1')
                f.write('\n      Class: rviz_default_plugins/RobotModel')
                f.write('\n      Collision Enabled: false')
                if robot.model == 'Crazyflie 2.1':
                    f.write('\n      Description File: /home/kiko/roboticpark_ws/src/uned_crazyflie_ros_pkg/uned_crazyflie_config/model/urdf/crazyflie_simple.urdf')
                else:
                    f.write('\n      Description File: /home/kiko/roboticpark_ws/src/uned_kheperaIV_ros_pkg/uned_khepera_description/urdf/khepera_simple.urdf')
                f.write('\n      Description Source: File')
                f.write('\n      Description Topic:')
                f.write('\n        Depth: 5')
                f.write('\n        Durability Policy: Volatile')
                f.write('\n        History Policy: Keep Last')
                f.write('\n        Reliability Policy: Reliable')
                f.write('\n        Value: ""')
                f.write('\n      Enabled: true')
                f.write('\n      Links:')
                f.write('\n        All Links Enabled: true')
                f.write('\n        Expand Joint Details: false')
                f.write('\n        Expand Link Details: false')
                f.write('\n        Expand Tree: false')
                f.write('\n        Link Tree Style: Links in Alphabetic Order')
                f.write('\n        base_link:')
                f.write('\n          Alpha: 1')
                f.write('\n          Show Axes: false')
                f.write('\n          Show Trail: false')
                f.write('\n          Value: true')
                f.write('\n      Mass Properties:')
                f.write('\n        Inertia: false')
                f.write('\n        Mass: false')
                f.write('\n      Name: '+robot.name)
                f.write('\n      TF Prefix: '+robot.name)
                f.write('\n      Update Interval: 0')
                f.write('\n      Value: true')
                f.write('\n      Visual Enabled: true')
                f.write('\n    - Alpha: 1')
                f.write('\n      Axes Length: 1')
                f.write('\n      Axes Radius: 0.10000000149011612')
                f.write('\n      Class: rviz_default_plugins/Pose')
                f.write('\n      Color: 255; 25; 0')
                f.write('\n      Enabled: true')
                f.write('\n      Head Length: 0.10000000149011612')
                f.write('\n      Head Radius: 0.05000000074505806')
                f.write('\n      Name: '+robot.name+'-local_pose')
                f.write('\n      Shaft Length: 0.25')
                f.write('\n      Shaft Radius: 0.019999999552965164')
                f.write('\n      Shape: Arrow')
                f.write('\n      Topic:')
                f.write('\n        Depth: 5')
                f.write('\n        Durability Policy: Volatile')
                f.write('\n        Filter size: 10')
                f.write('\n        History Policy: Keep Last')
                f.write('\n        Reliability Policy: Reliable')
                f.write('\n        Value: /'+robot.name+'/local_pose')
                f.write('\n      Value: true')
                f.write('\n    - Alpha: 1')
                f.write('\n      Axes Length: 1')
                f.write('\n      Axes Radius: 0.10000000149011612')
                f.write('\n      Class: rviz_default_plugins/Pose')
                f.write('\n      Color: 255; 163; 72')
                f.write('\n      Enabled: true')
                f.write('\n      Head Length: 0.15000000596046448')
                f.write('\n      Head Radius: 0.05000000074505806')
                f.write('\n      Name: '+robot.name+'-goal_pose')
                f.write('\n      Shaft Length: 0.25')
                f.write('\n      Shaft Radius: 0.02500000037252903')
                f.write('\n      Shape: Arrow')
                f.write('\n      Topic:')
                f.write('\n        Depth: 5')
                f.write('\n        Durability Policy: Volatile')
                f.write('\n        Filter size: 10')
                f.write('\n        History Policy: Keep Last')
                f.write('\n        Reliability Policy: Reliable')
                f.write('\n        Value: /'+robot.name+'/goal_pose')
                f.write('\n      Value: true')
                if robot.communication_path == 'true':
                    f.write('\n    - Alpha: 1')
                    f.write('\n      Buffer Length: 1')
                    f.write('\n      Class: rviz_default_plugins/Path')
                    f.write('\n      Color: 26; 95; 180')
                    f.write('\n      Enabled: true')
                    f.write('\n      Head Diameter: 0.30000001192092896')
                    f.write('\n      Head Length: 0.20000000298023224')
                    f.write('\n      Length: 0.30000001192092896')
                    f.write('\n      Line Style: Billboards')
                    f.write('\n      Line Width: 0.029999999329447746')
                    f.write('\n      Name: '+robot.name+'-path')
                    f.write('\n      Offset:')
                    f.write('\n        X: 0')
                    f.write('\n        Y: 0')
                    f.write('\n        Z: 0')
                    f.write('\n      Pose Color: 255; 85; 255')
                    f.write('\n      Pose Style: None')
                    f.write('\n      Radius: 0.029999999329447746')
                    f.write('\n      Shaft Diameter: 0.10000000149011612')
                    f.write('\n      Shaft Length: 0.10000000149011612')
                    f.write('\n      Topic:')
                    f.write('\n        Depth: 5')
                    f.write('\n        Durability Policy: Volatile')
                    f.write('\n        Filter size: 10')
                    f.write('\n        History Policy: Keep Last')
                    f.write('\n        Reliability Policy: Reliable')
                    f.write('\n        Value: /'+robot.name+'/path')
                    f.write('\n      Value: true')
                aux0 = robot.task_relationship.split(', ')
                for rel in aux0:
                    aux1 = rel.split('_')
                    id = aux1[0]
                    f.write('\n    - Class: rviz_default_plugins/Marker')
                    f.write('\n      Enabled: true')
                    f.write('\n      Name: '+robot.name+'-'+id)
                    f.write('\n      Namespaces:')
                    f.write('\n        { }')
                    f.write('\n      Topic:')
                    f.write('\n        Depth: 5')
                    f.write('\n        Durability Policy: Volatile')
                    f.write('\n        Filter size: 10')
                    f.write('\n        History Policy: Keep Last')
                    f.write('\n        Reliability Policy: Reliable')
                    f.write('\n        Value: /'+robot.name+'/'+id+'/marker')
                    f.write('\n      Value: true')

        f.write('\n  Enabled: true')
        f.write('\n  Global Options:')
        f.write('\n    Background Color: 80; 111; 93')
        f.write('\n    Fixed Frame: map')
        f.write('\n    Frame Rate: 30')
        f.write('\n  Name: root')
        f.write('\n  Tools:')
        f.write('\n    - Class: rviz_default_plugins/Interact')
        f.write('\n      Hide Inactive Objects: true')
        f.write('\n    - Class: rviz_default_plugins/MoveCamera')
        f.write('\n    - Class: rviz_default_plugins/Select')
        f.write('\n    - Class: rviz_default_plugins/FocusCamera')
        f.write('\n    - Class: rviz_default_plugins/Measure')
        f.write('\n      Line color: 128; 128; 0')
        f.write('\n    - Class: rviz_default_plugins/SetInitialPose')
        f.write('\n      Covariance x: 0.25')
        f.write('\n      Covariance y: 0.25')
        f.write('\n      Covariance yaw: 0.06853891909122467')
        f.write('\n      Topic:')
        f.write('\n        Depth: 5')
        f.write('\n        Durability Policy: Volatile')
        f.write('\n        History Policy: Keep Last')
        f.write('\n        Reliability Policy: Reliable')
        f.write('\n        Value: /initialpose')
        f.write('\n    - Class: rviz_default_plugins/SetGoal')
        f.write('\n      Topic:')
        f.write('\n        Depth: 5')
        f.write('\n        Durability Policy: Volatile')
        f.write('\n        History Policy: Keep Last')
        f.write('\n        Reliability Policy: Reliable')
        f.write('\n        Value: origin/local_pose')
        f.write('\n    - Class: rviz_default_plugins/PublishPoint')
        f.write('\n      Single click: true')
        f.write('\n      Topic:')
        f.write('\n        Depth: 5')
        f.write('\n        Durability Policy: Volatile')
        f.write('\n        History Policy: Keep Last')
        f.write('\n        Reliability Policy: Reliable')
        f.write('\n        Value: /clicked_point')
        f.write('\n  Transformation:')
        f.write('\n    Current:')
        f.write('\n      Class: rviz_default_plugins/TF')
        f.write('\n  Value: true')
        f.write('\n  Views:')
        f.write('\n    Current:')
        f.write('\n      Class: rviz_default_plugins/Orbit')
        f.write('\n      Distance: 4.903817176818848')
        f.write('\n      Enable Stereo Rendering:')
        f.write('\n        Stereo Eye Separation: 0.05999999865889549')
        f.write('\n        Stereo Focal Distance: 1')
        f.write('\n        Swap Stereo Eyes: false')
        f.write('\n        Value: false')
        f.write('\n      Focal Point:')
        f.write('\n        X: -0.08397747576236725')
        f.write('\n        Y: -0.17901606857776642')
        f.write('\n        Z: 0.5602047443389893')
        f.write('\n      Focal Shape Fixed Size: true')
        f.write('\n      Focal Shape Size: 0.05000000074505806')
        f.write('\n      Invert Z Axis: false')
        f.write('\n      Name: Current View')
        f.write('\n      Near Clip Distance: 0.009999999776482582')
        f.write('\n      Pitch: 0.7353980541229248')
        f.write('\n      Target Frame: <Fixed Frame>')
        f.write('\n      Value: Orbit (rviz)')
        f.write('\n      Yaw: 2.6654012203216553')
        f.write('\n    Saved: ~')
        f.write('\nWindow Geometry:')
        f.write('\n  Displays:')
        f.write('\n    collapsed: true')
        f.write('\n  Height: 846')
        f.write('\n  Hide Left Dock: true')
        f.write('\n  Hide Right Dock: false')
        f.write('\n  QMainWindow State: 000000ff00000000fd0000000400000000000001ae000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073000000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002fb00fffffffb0000000800540069006d00650100000000000004500000000000000000000004b0000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000')
        f.write('\n  Selection:')
        f.write('\n    collapsed: false')
        f.write('\n  Time:')
        f.write('\n    collapsed: false')
        f.write('\n  Tool Properties:')
        f.write('\n    collapsed: false')
        f.write('\n  Views:')
        f.write('\n    collapsed: false')
        f.write('\n  Width: 1200')
        f.write('\n  X: 64')
        f.write('\n  Y: 176')
        f.close()


if __name__ == "__main__":
    app =  QtWidgets.QApplication(sys.argv)
    window = MyApp()
    window.show()
    app.exec_()