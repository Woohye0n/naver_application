#!/usr/bin/python

import rospy
from visualization_msgs.msg import Marker
from rr_common.srv import MapWayPointXY
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np

class MapCourseVisualizer:
    def __init__(self):
        self.pub_in = rospy.Publisher("~highway_in", Marker, queue_size=10)
        self.pub_out = rospy.Publisher("~highway_out", Marker, queue_size=10)
        
        self.highway_out_points = Marker()
        self.highway_out_points.type = self.highway_out_points.SPHERE_LIST
        self.highway_out_points.header.frame_id = "map"
        self.highway_out_points.color.a = 1.0
        self.highway_out_points.color.r = 0
        self.highway_out_points.color.g = 1
        self.highway_out_points.color.b = 0
        self.highway_out_points.scale.x = 0.5
        self.highway_out_points.scale.y = 0.5
        self.highway_out_points.scale.z = 0.5
        self.highway_out_points.pose.position.x = 0
        self.highway_out_points.pose.position.y = 0
        self.highway_out_points.pose.position.z = 0
        self.highway_out_points.pose.orientation.x = 0
        self.highway_out_points.pose.orientation.y = 0
        self.highway_out_points.pose.orientation.z = 0
        self.highway_out_points.pose.orientation.w = 1

        self.highway_in_points = Marker()
        self.highway_in_points.type = self.highway_in_points.SPHERE_LIST
        self.highway_in_points.header.frame_id = "map"
        self.highway_in_points.color.a = 1.0
        self.highway_in_points.color.r = 0
        self.highway_in_points.color.g = 1
        self.highway_in_points.color.b = 0
        self.highway_in_points.scale.x = 0.5
        self.highway_in_points.scale.y = 0.5
        self.highway_in_points.scale.z = 0.5
        self.highway_in_points.pose.position.x = 0
        self.highway_in_points.pose.position.y = 0
        self.highway_in_points.pose.position.z = 0
        self.highway_in_points.pose.orientation.x = 0
        self.highway_in_points.pose.orientation.y = 0
        self.highway_in_points.pose.orientation.z = 0
        self.highway_in_points.pose.orientation.w = 1

    def map_client(self):
        rospy.wait_for_service('~map')
        try:
            print('try')
            map = rospy.ServiceProxy('~map', MapWayPointXY)
            header = Header()
            header.frame_id = 'rr_vis'
            res = map(header)
            self.highway_in = res.highway_in
            self.highway_out = res.highway_out
            print('service called')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
    def publish(self):
        in_points = np.array(self.highway_in).reshape((-1,2))
        out_points = np.array(self.highway_out).reshape((-1,2))
        for pt in in_points:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            self.highway_in_points.points.append(p)
        for pt in out_points:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            self.highway_out_points.points.append(p)

        self.pub_in.publish(self.highway_in_points)
        self.pub_out.publish(self.highway_out_points)
        


def main():
    rospy.init_node("rr_vis_map_course", anonymous=True)
    vis = MapCourseVisualizer()
    vis.map_client()
    rate = rospy.Rate(20)  # 20hz
    while not rospy.is_shutdown():
        vis.publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
