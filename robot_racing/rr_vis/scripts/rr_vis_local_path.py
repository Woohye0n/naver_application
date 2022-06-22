#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker
from rr_common.msg import LocalPathPoints
from geometry_msgs.msg import Point
import numpy as np


class LocalPathVisualizer:
    def __init__(self):
        self.pub_path = rospy.Publisher("~local_marker", Marker, queue_size=10)

        self.sub = rospy.Subscriber("~local_path", LocalPathPoints, self.pathcb, queue_size=3)

        self.sphere_list = Marker()
        self.sphere_list.type = self.sphere_list.SPHERE_LIST
        self.sphere_list.header.frame_id = "map"
        self.sphere_list.color.a = 1.0
        self.sphere_list.color.r = 0
        self.sphere_list.color.g = 0
        self.sphere_list.color.b = 1
        self.sphere_list.scale.x = 1
        self.sphere_list.scale.y = 1
        self.sphere_list.scale.z = 1
        self.sphere_list.pose.position.x = 0
        self.sphere_list.pose.position.y = 0
        self.sphere_list.pose.position.z = 0
        self.sphere_list.pose.orientation.x = 0
        self.sphere_list.pose.orientation.y = 0
        self.sphere_list.pose.orientation.z = 0
        self.sphere_list.pose.orientation.w = 1

    def pathcb(self, data):
        self.sphere_list.points=[]
        points = np.array(data.local_path_points).reshape((-1,2))
        for pt in points:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            self.sphere_list.points.append(p)
            
    def publish(self):
        self.pub_path.publish(self.sphere_list)


def main():
    rospy.init_node("rr_vis_local_path", anonymous=True)
    vis = LocalPathVisualizer()
    rate = rospy.Rate(20)  # 20hz
    while not rospy.is_shutdown():
        vis.publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
