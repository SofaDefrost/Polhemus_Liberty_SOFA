#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pyhton library
import threading
from numpy import abs, sign

# ros2 import 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# polhemus functions
from .submodules.PolhemusUSB import *

class Position_echelon3(Node):
    """
    Init the node to get position in the echelon3 referential
    create a threading for the rate     
    """

    def __init__(self,rosName,rate=100.0,numberOfSensors=1):
        """
        Parameters:
            rosName : name for the node
            rate : rate publication
            numberOfSensors : number of sensors attached
        """

        # Init ROS2 node
        Node.__init__(self,rosName)

        # init thread and rate
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        self.rosRate = self.create_rate(rate)

        # declare parameters
        self.declare_parameter('rate', rate)
        self.declare_parameter('numberOfSensors', numberOfSensors)

        # get parameters
        self.numSensors = self.get_parameter('numberOfSensors').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value


        # init publishers
        self.initgrah()

        # init polhemus
        self.pol = PolhemusUSB(self.numSensors)
        self.scale = 10 # convert cm to mm

        # get init position
        self.pol.UpdateSensors()
        position = self.pol.sensors[0].GetLastPosition()
        self.offset = position  
        self.initPose = [758.45,0,0] #This is the position of the end effector of echelon but more can be use if more sensor

    def initgrah(self):
        # init publishers
        self.pub = [self.create_publisher(Pose,'polhemus/sensor'+str(i),qos_profile=10) 
                    for i in range(self.numSensors)]           
        self.poses = [Pose() for i in range(self.numSensors)]
        
    ################ position functions ################
            
    def UpdateState(self):
        """Update the state sensor and the topic value"""
        self.pol.UpdateSensors()
        for i in range(self.numSensors):
            position = self.pol.sensors[i].GetLastPosition()
            if position[0]>=0 :
                self.poses[i].position.x = self.initPose[0] + self.scale*(position[0] -self.offset[0])
                self.poses[i].position.y = self.initPose[1] -self.scale*(position[1] -self.offset[1])
                self.poses[i].position.z = self.initPose[2] -self.scale*(position[2] -self.offset[2])
            else :
                self.poses[i].position.x = self.initPose[0] -self.scale*position[0] 
                self.poses[i].position.y = self.initPose[1] - self.scale*position[1] 
                self.poses[i].position.z = self.initPose[2] - self.scale*position[2] 
           
            quat = self.pol.sensors[i].GetLastQuaternion()
            self.poses[i].orientation.x = quat[1]
            self.poses[i].orientation.y = quat[2]
            self.poses[i].orientation.z = quat[3]
            self.poses[i].orientation.w = quat[0]

    def UpdateLoop(self):
        """
        Publishes the topic with the state of each sensor
        """
        while rclpy.ok():
            self.UpdateState()
            for i in range(self.numSensors):
                self.pub[i].publish(self.poses[i])
            self.rosRate.sleep()
            
        self.destroy_node()
        

#########################################
# main_function
######################################### 

def main():

    rclpy.init()
    position = Position_echelon3("PositionRosNode")
    position.UpdateLoop()
    rclpy.shutdown()
