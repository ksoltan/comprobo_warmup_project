#!/usr/bin/env python
"""
This is a teleoperation script which allows you to control the robot, as well as change its state or mission. <- maybe this is better in another node?
"""
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopNode(object):
    def __init__(self):
        # initialize teleop node
        rospy.init_node("neato_teleop")

        # initialize publisher to cmd_vel
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.key_pressed = None
        self.command_entered = None
        self.settings = termios.tcgetattr(sys.stdin)
        # TODO: initialize publisher to state machine


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        # return selfkey

    def run(self):
        # Wait for a key to be pressed. Then, update the cmd_vel.
        while self.key_pressed != '\x03':
            self.getKey()
            # TODO: may need a separate thread to concurrently keep publishing cmd_vel. Or set a timeout
            if(self.key_pressed == ':'):
                print(":"), # Print : on same line as input entry
                self.command_entered = raw_input()
                print "Entered command :{}".format(self.command_entered)
            else:
                print "Pressed this key: {}".format(self.key_pressed)

if __name__ == "__main__":
    node = TeleopNode()
    node.run()
