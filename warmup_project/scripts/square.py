#!/usr/bin/env python
"""
Make the Neato square dance.
"""

import rospy
from warmup_project.msg import LabeledPolarVelocity2D, PolarVelocity2D, State
from dynamic_reconfigure.server import Server
from warmup_project.cfg import SquareConfig

class SquareDance(object):
    def __init__(self):
        behavior = "square_dance"
        rospy.init_node(behavior + "_node")

        self.server = Server(SquareConfig, self.config_callback)

        self.behavior_id = State.SQUARE_DANCE

        # Setup publisher/subscription
        self.publisher_cmd_vel = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10, latch=True)

    def config_callback(self, config, level):
        self.forward_wait_time = config.forward_wait_time
        self.turn_wait_time = config.turn_wait_time
        return config

    def square_side(self):
        # Move forwards at 1 m/s
        print("Move forward")
        cmd_vel = LabeledPolarVelocity2D(
            node_ID=self.behavior_id, 
            velocity=PolarVelocity2D(linear=1, angular=0))
        self.publisher_cmd_vel.publish(cmd_vel)
        
        # Wait 1s
        rospy.sleep(self.forward_wait_time)

        # Stop and turn
        print("Turn")
        cmd_vel = LabeledPolarVelocity2D(
            node_ID=self.behavior_id, 
            velocity=PolarVelocity2D(linear=0, angular=10))
        self.publisher_cmd_vel.publish(cmd_vel)

        # Wait 1s
        rospy.sleep(self.turn_wait_time)

    def run(self):
        while not rospy.is_shutdown():
            # self.forward_wait_time = rospy.get_param('forward_wait_time')
            # self.turn_wait_time = rospy.get_param('turn_wait_time')
            self.square_side()

if __name__ == "__main__":
    node = SquareDance()
    node.run()