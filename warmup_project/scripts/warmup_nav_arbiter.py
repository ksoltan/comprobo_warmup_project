#!/usr/bin/env python
"""
The warmup nav arbiter handles keeping track of the state of the Neato and routing
the appropriate velocity commands. All commands coming from the Teleop program take
precedence. If the robot hits an estop mode, there is a slight grace period that allows
the robot to be moved out of the situation.
"""
import rospy
from warmup_project.msg import LabeledPolarVelocity2D, PolarVelocity2D, State
from geometry_msgs.msg import Twist

# TODO:Implement grace period for the robot to move backwards. POWER MOVE.

class WarmupNavArbiter(object):
    def __init__(self):
        rospy.init_node("warmup_nav_arbiter_node")
        self.desired_vel_subscriber = rospy.Subscriber("/desired_cmd_vel", LabeledPolarVelocity2D, self.update_current_request)
        self.desired_state_subscriber = rospy.Subscriber("/desired_state", State, self.update_state)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.state = State.TELEOP
        self.current_requesting_node = None
        self.desired_vel = PolarVelocity2D(linear=0, angular=0)

    def update_current_request(self, desired_vel_msg):
        # print "Updating a request."
        self.current_requesting_node = desired_vel_msg.node_ID # While using node names and state names identically
        self.desired_vel = desired_vel_msg.velocity
        # The TELEOP is special line (TELEOP takes precedence over all other states)
        if self.current_requesting_node == State.TELEOP:
            self.state = State.TELEOP
        self.set_desired_vel()

    def update_state(self, state_msg):
        # print "Updating state to {}".format(state_msg.state)
        self.state = state_msg.state

    def set_desired_vel(self):
        print "Requesting node: {}\t Current state: {}".format(self.current_requesting_node, self.state)
        if self.current_requesting_node == self.state:
            # convert the polar velocity to Twist msg
            twist_msg = Twist()
            twist_msg.linear.x = self.desired_vel.linear
            twist_msg.angular.z = self.desired_vel.angular

            # publish this twist message
            self.cmd_vel_publisher.publish(twist_msg)

    def run(self):
        print "Hi! I'm your arbiter! Let me route your velocities."
        rospy.spin()

if __name__ == "__main__":
    node = WarmupNavArbiter()
    node.run()
