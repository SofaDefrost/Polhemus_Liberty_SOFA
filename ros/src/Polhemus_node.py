#!/usr/bin/env python

from logging import error
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import PolhemusUSB

scale = 0.01 # cm to m conversion

class Polhemus_node(object):
    """
    docstring
    """
    def __init__(self,rate = 10,numberOfSensors = 4):
        """
        class Polhemus_node(object):
        Creates a Ros node that publishes the pose of each sensor
        uses the following ros parameters :
            ~rate : rate of publication
            ~numberOfSensors : number of poses to publish
        """
        rospy.init_node('Polhemus_sensor', anonymous=True)
        rate = rospy.get_param("~rate",rate)
        rospy.set_param("~rate",rate)
        self.rate = rospy.Rate(rate)  

        numberOfSensors = rospy.get_param("~numberOfSensors",numberOfSensors)
        rospy.set_param("~numberOfSensors",numberOfSensors)
        self.num = numberOfSensors
        if self.num>4:
            error("numberOfSensors has to be <= 4")
            self.num = 4
        
        self.pub = [
                rospy.Publisher(
                'polhemus/sensor'+str(i), 
                geometry_msgs.msg.PoseStamped, 
                queue_size=10) 
                for i in range(self.num)]
        
        self.poses = [geometry_msgs.msg.PoseStamped() for _ in range(self.num)]
        for pose in self.poses:
            pose.header.frame_id="base"

        self.pol = PolhemusUSB.PolhemusUSB()
    
    def UpdateLoop(self):
        """
        Publishes the topic with the state of each motor
        """
        while not rospy.is_shutdown():
            self.UpdateState()
            for i in range(self.num):
                self.pub[i].publish(self.poses[i])
            self.rate.sleep()
    
    def UpdateState(self):
        self.pol.UpdateSensors()
        for i in range(self.num):
            self.poses[i].header.stamp = rospy.Time.now()
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
        

    pass

if __name__ == "__main__":  
    p = Polhemus_node()
    p.UpdateLoop()