#!/usr/bin/env python
""" This script publishes a marker message for visualization in rviz. """

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
# from std_msgs.msg import ColorRGBA

class MarkerNode(object):
    def __init__(self):
        # init node
        rospy.init_node("test_marker_node")
        # init marker msgs
        self.my_marker = Marker()
        self.my_marker.header.frame_id = "odom"
        self.my_marker.type = Marker.SPHERE
        self.my_marker.pose.position.x = 1
        self.my_marker.pose.position.y = 2
        self.my_marker.scale = Vector3(1, 1, 1)
        self.my_marker.color.g = 0.9
        self.my_marker.color.a = 0.5
        # init publisher
        self.marker_publisher = rospy.Publisher("/visualization_marker",
                                                Marker, queue_size=10)

    def run(self):
        # Publish marker message at 10Hz
        r = rospy.Rate(10)
        print("Starting run.")
        while not rospy.is_shutdown():
            # Update timestamp and publish
            self.my_marker.header.stamp = rospy.Time.now()
            self.marker_publisher.publish(self.my_marker)

if __name__ == "__main__":
    node = MarkerNode()
    node.run()
