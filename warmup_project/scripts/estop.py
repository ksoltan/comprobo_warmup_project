#!/usr/bin/env python
""" This program waits for the Neato to bump into something, and then publishes an estop state."""

from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from warmup_project.msg import LabeledPolarVelocity2D, PolarVelocity2D, State
import rospy

class EstopBehavior(object):
    def __init__(self):
        rospy.init_node("estop_node")
        self.cmd_vel_publisher = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10, latch=True)
        self.state_publisher = rospy.Publisher("/desired_state", State, queue_size=10)
        self.bump_subscriber = rospy.Subscriber("/bump", Bump, self.estop)

    def estop(self, bump_msg):
        # If any of the bump sensors are depressed, STAHP.
        if(bump_msg.leftSide or bump_msg.leftFront or bump_msg.rightSide or bump_msg.rightFront):
            print("EMERGENCY. STAHHHHP")
            self.cmd_vel_publisher.publish(LabeledPolarVelocity2D(node_ID=str(State.ESTOP), velocity=PolarVelocity2D(linear=0, angular=0)))
            self.state_publisher.publish(State.ESTOP)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = EstopBehavior()
    node.run()
