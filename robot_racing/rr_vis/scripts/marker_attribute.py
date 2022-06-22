#!/usr/bin/python

from copy import deepcopy
import rospy
from visualization_msgs.msg import Marker, MarkerArray


class MarkerGenerator:
    def __init__(self):
        self.param = {}
        self.default_marker = Marker()

    def set_marker_defaults(self, marker_type):
        self.default_marker.type = marker_type
        self.default_marker.header.frame_id = rospy.get_param(
            "~frame_id", default="map"
        )
        self.default_marker.ns = rospy.get_param("~ns", default="")
        self.default_marker.lifetime = rospy.Time(
            rospy.get_param("~lifetime", default=0.0)
        )
        self.default_marker.color.a = rospy.get_param("~color_a", default=1.0)
        self.default_marker.color.r = rospy.get_param("~color_r", default=1.0)
        self.default_marker.color.g = rospy.get_param("~color_g", default=1.0)
        self.default_marker.color.b = rospy.get_param("~color_b", default=1.0)
        self.default_marker.scale.x = rospy.get_param("~scale_x", default=0.5)
        self.default_marker.scale.y = rospy.get_param("~scale_y", default=0.5)
        self.default_marker.scale.z = rospy.get_param("~scale_z", default=0.5)

    def new_marker(self, stamp=None):
        new_marker = deepcopy(self.default_marker)
        if stamp:
            new_marker.header.stamp = stamp
        else:
            new_marker.header.stamp = rospy.Time.now()
        return new_marker
