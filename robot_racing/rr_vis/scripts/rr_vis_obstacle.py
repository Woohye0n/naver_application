#!/usr/bin/python

import rospy
from marker_attribute import MarkerGenerator, MarkerArray, Marker
from rospy.core import rospyinfo
from rr_common.msg import PerceptionObstacleArray, PerceptionObstacle
import math


class ObstacleVisualizer:
    def __init__(self):
        self.msg = MarkerArray()
        self.mrk_gen = MarkerGenerator()
        self.mrk_gen.set_marker_defaults(Marker.CUBE)
        self.arrow_gen = MarkerGenerator()
        self.arrow_gen.set_marker_defaults(Marker.ARROW)
        self.sub = rospy.Subscriber(
            "~obstacles", PerceptionObstacleArray, self.callback, queue_size=3
        )
        self.pub = rospy.Publisher("~marker", MarkerArray, queue_size=3)

    def callback(self, msg):
        self.msg.markers = []
        for obs in msg.obstacles:
            m = self.mrk_gen.new_marker()
            m.id = obs.id
            m.pose = obs.pose
            m.scale.x = obs.shape.dimensions[0]
            m.scale.y = obs.shape.dimensions[1]
            m.scale.z = obs.shape.dimensions[2]
            self.msg.markers.append(m)

            a = self.arrow_gen.new_marker()
            a.id = obs.id + len(msg.obstacles)

            v = (
                obs.twist.linear.x * obs.twist.linear.x
                + obs.twist.linear.y * obs.twist.linear.y
                + obs.twist.linear.z * obs.twist.linear.z
            ) ** 0.5
            v_th = math.atan2(obs.twist.linear.y, obs.twist.linear.x)

            a.pose.position = obs.pose.position
            a.pose.orientation.x = 0
            a.pose.orientation.y = 0
            a.pose.orientation.z = math.sin(v_th / 2)
            a.pose.orientation.w = math.cos(v_th / 2)

            a.scale.x = v
            a.scale.y = 0.1
            a.scale.z = 0.1
            self.msg.markers.append(a)

    def publish(self):
        self.pub.publish(self.msg)


def main():
    rospy.init_node("rr_vis_obstacle")
    vis = ObstacleVisualizer()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        vis.publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
