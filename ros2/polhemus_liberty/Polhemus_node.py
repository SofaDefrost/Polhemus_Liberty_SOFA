#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pyhton library
from logging import error
import threading

# ros2 import 
import rclpy
from rclpy.node import Node
import geometry_msgs.msg

# polhemus functions
from .submodules.PolhemusUSB import *

scale = 10 # cm to mm conversion

class Polhemus_node(Node,object):
    """
    Class for Polhemus to read data (using PolhemusUSB.py)
    """
    def __init__(self,rosName, rate = 100.0, numberOfSensors = 1):
        """
        class Polhemus_node(object):
        Creates a Ros2 node that publishes the pose of each sensor
        Parameters :
            rosName : node name
            rate : rate of publication in Hz
            numberOfSensors : number of poses to publish
        """
        # init node
        Node.__init__(self,rosName)

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        
        # declare parameters
        self.declare_parameter('rate', rate)
        self.declare_parameter('numberOfSensors', numberOfSensors)

        # get parameter value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.numSensors = self.get_parameter('numberOfSensors').get_parameter_value().integer_value

        # create rate
        self.rosRate = self.create_rate(rate)  

        if self.numSensors>4:
            error("numberOfSensors has to be <= 4")
            self.numSensors = 4
        
        self.initgraph()

        self.pol = PolhemusUSB(self.numSensors)
    

    def initgraph(self):
        """Init the publishers and the subscribersS"""
        self.pub = [
                self.create_publisher(
                    geometry_msgs.msg.PoseStamped,
                    'polhemus/sensor'+str(i),  
                    qos_profile=10) 
                    for i in range(self.numSensors)
                    ]
        
        self.poses = [geometry_msgs.msg.PoseStamped() for _ in range(self.numSensors)]
        for pose in self.poses:
            pose.header.frame_id="base"

       
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

    def UpdateState(self):
        """Update the state sensor and the topic value"""
        self.pol.UpdateSensors()
        for i in range(self.numSensors):
            self.poses[i].header.stamp = self.get_clock().now().to_msg()
            position = self.pol.sensors[i].GetLastPosition()
            self.poses[i].pose.position.x, \
            self.poses[i].pose.position.y, \
            self.poses[i].pose.position.z  \
              = [coord*scale for coord in self.pol.sensors[i].GetLastPosition()]
            quat = self.pol.sensors[i].GetLastQuaternion()
            
            self.poses[i].pose.orientation.x = quat[1]
            self.poses[i].pose.orientation.y = quat[2]
            self.poses[i].pose.orientation.z = quat[3]
            self.poses[i].pose.orientation.w = quat[0]
        


def main():
    rclpy.init()
    p = Polhemus_node('PolhemusRosNode')
    p.UpdateLoop()
    rclpy.shutdown()