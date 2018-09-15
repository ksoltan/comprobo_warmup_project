#!/usr/bin/env python
"""
The follow arbiter handles choosing the target object for the robot to follow,
and combines the angle and distance corrections into one cmd_vel.
This arbiter will prioritize following people. If it does not find a person,
it will find the nearest wall.
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from warmup_project.msg import LabeledPolarVelocity2D, PolarVelocity2D, State

class FollowArbiter(object):
    def __init__(self):
        rospy.init_node("follow_arbiter_node")
        self.behavior_id = State.FOLLOW
        # Setup publisher to announce the target to follow and the desired_cmd_vel of the follower.
        self.target_publisher = rospy.Publisher("/target_follow", Twist, queue_size=10)
        self.desired_vel_publisher = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10)

        # Setup subscriber to read the angle and distance correction cmds, which it will then concatenate somehow together...
        self.desired_vel_publisher = rospy.Subscriber("/desired_cmd_vel", LabeledPolarVelocity2D, self.update_cmd_vel_correction)
        self.angular_correction = None
        self.distance_correction = None

        # Setup subscriber to scan message from lidar
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.update_scan_ranges)
        self.scan_ranges = None

    def update_scan_ranges(self, scan_msg):
        # print "Updating state to {}".format(state_msg.state)
        self.scan_msg = state_msg.ranges

    def update_cmd_vel_correction(self, cmd_vel_msg):
        if cmd_vel_msg.node_ID == State.MAINTAIN_ANGLE:
            self.angular_correction = cmd_vel_msg.velocity
        if cmd_vel_msg.node_ID == State.MAINTAIN_DISTANCE:
            self.distance_correction = cmd_vel_msg.velocity

    '''Based on the scan, identify the target of interest'''
    def update_target(self):

    '''Based on the follow state, smash the two corrections together somehow...?'''
    # This may be a good thing to abstract into a class that is specific to the thing you are following.
    # Would it make sense to decide what you want to be tracking in this arbiter, and then depending on that
    def update_vel(self):


    def run(self):
        print "Hi! I'm your follow arbiter! Let me decide what object you shouldn't run over (or should)."
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_target()
            self.update_vel()
            r.sleep() # Calculate error every 10 seconds.
        rospy.spin()

if __name__ == "__main__":
    node = WarmupNavArbiter()
    node.run()
