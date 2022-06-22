#!/usr/bin/python

from math import pi, cos, sin
import rospy
from visualization_msgs.msg import Marker
from rr_common.msg import StateEstimated
import numpy as np

class PositionVisualizer:
    def __init__(self):
        self.pub_position = rospy.Publisher("~position_marker", Marker, queue_size=10)
        self.pub_heading = rospy.Publisher("~heading_arrow", Marker, queue_size=10)
        self.pub_box = rospy.Publisher("~box", Marker, queue_size=10)

        self.sub = rospy.Subscriber("~state", StateEstimated, self.statecb, queue_size=3)

        self.sphere = Marker()
        self.sphere.type = self.sphere.SPHERE
        self.sphere.header.frame_id = "map"
        self.sphere.color.a = 1.0
        self.sphere.color.r = 1
        self.sphere.color.g = 0
        self.sphere.color.b = 0
        self.sphere.scale.x = 1.3
        self.sphere.scale.y = 1.3
        self.sphere.scale.z = 1.3
        self.sphere.pose.orientation.x = 0
        self.sphere.pose.orientation.y = 0
        self.sphere.pose.orientation.z = 0
        self.sphere.pose.orientation.w = 1

        self.heading_arrow = Marker()
        self.heading_arrow.type = self.heading_arrow.ARROW
        self.heading_arrow.header.frame_id = "map"
        self.heading_arrow.color.a = 1.0
        self.heading_arrow.color.r = 0.5
        self.heading_arrow.color.g = 0
        self.heading_arrow.color.b = 0.5
        self.heading_arrow.scale.x = 5
        self.heading_arrow.scale.y = 0.5
        self.heading_arrow.scale.z = 0.3

        self.box = Marker()
        self.box.type = self.box.CUBE
        self.box.header.frame_id = "map"
        self.box.color.a = 0.8
        self.box.color.r = 0.5
        self.box.color.g = 0.5
        self.box.color.b = 0.5
        self.box.scale.x = 2.06
        self.box.scale.y = 1.14
        self.box.scale.z = 0.1
    
    def statecb(self, data):
        #current point sphere
        cur_point = np.array(data.current_xy).reshape((-1,2))
        self.sphere.pose.position.x = cur_point[0][0]
        self.sphere.pose.position.y = cur_point[0][1]
        self.sphere.pose.position.z = 0
        self.heading_arrow.pose.position.x = cur_point[0][0]
        self.heading_arrow.pose.position.y = cur_point[0][1]
        self.heading_arrow.pose.position.z = 0
        
        #current heading arrow
        heading = data.heading
        theta = pi * 0.5 - heading * pi / 180 
        self.heading_arrow.pose.orientation.x = 0
        self.heading_arrow.pose.orientation.y = 0
        self.heading_arrow.pose.orientation.z = sin(theta / 2)
        self.heading_arrow.pose.orientation.w = cos(theta / 2)

        #currnet box
        self.box.pose.position.x = cur_point[0][0] + cos(theta) * 0.84
        self.box.pose.position.y = cur_point[0][1] + sin(theta) * 0.84
        self.box.pose.position.z = 0
        self.box.pose.orientation.x = 0
        self.box.pose.orientation.y = 0
        self.box.pose.orientation.z = sin(theta / 2)
        self.box.pose.orientation.w = cos(theta / 2)

    def publish(self):
        self.pub_position.publish(self.sphere)
        self.pub_heading.publish(self.heading_arrow)
        self.pub_box.publish(self.box)
      
def main():
    rospy.init_node("rr_vis_position", anonymous=True)
    vis = PositionVisualizer()
    rate = rospy.Rate(20)  # 20hz
    while not rospy.is_shutdown():
        vis.publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
