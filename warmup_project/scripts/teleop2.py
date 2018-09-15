#!/usr/bin/env python
"""
This is a teleoperation script which allows you to control the robot, as well as change its state or mission.
The script runs as a standalone node, bypassing the higher warmup_nav_arbiter, and only updating its state.
"""
import rospy
from warmup_project.msg import LabeledPolarVelocity2D, PolarVelocity2D, State
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Define mapping of command names to robot states
command_to_state = {
"estop" : State.ESTOP,
"wall": State.WALL_FOLLOW,
"ang": State.MAINTAIN_ANGLE,
"dist": State.MAINTAIN_DISTANCE,
"fol": State.FOLLOW
}

key_actions = {
'a': 'MOVE_LEFT',
'd': 'MOVE_RIGHT',
'w': 'MOVE_FORWARD',
's': 'MOVE_BACKWARD',
'z': "TURN_LEFT",
'x': "TURN_RIGHT",
'0': "E_STOP"
}

action_bindings = {
'MOVE_LEFT' : PolarVelocity2D(linear=0.5, angular=1),
'MOVE_RIGHT' : PolarVelocity2D(linear=0.5, angular=-1),
'MOVE_FORWARD' : PolarVelocity2D(linear=1, angular=0),
'MOVE_BACKWARD' : PolarVelocity2D(linear=-1, angular=0),
"TURN_LEFT" : PolarVelocity2D(linear=0, angular=1),
"TURN_RIGHT" : PolarVelocity2D(linear=0, angular=-1),
"E_STOP" : PolarVelocity2D(linear=0, angular=0)
}

class TeleopNode(object):
    def __init__(self):
        # initialize teleop node
        rospy.init_node("neato_teleop")
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.state_publisher = rospy.Publisher("/desired_state", State, queue_size=10)
        self.target_publisher = rospy.Publisher("/target_follow", Twist, queue_size=10)
        self.ang_target = 0
        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        # Wait for a key to be pressed. Then, update the cmd_vel.
        while self.key_pressed != '\x03':
            self.getKey()

            # Check for a behavior command if user enters a colon ':'
            if(self.key_pressed == ':'):
                print(":"), # Print : on same line as input entry
                new_command = raw_input()
                # Set state to new state if the state has changed.
                if(new_command in command_to_state.keys()):
                    self.state_publisher.publish(command_to_state[new_command])
                    print "Commanded state: {}".format(command_to_state[new_command])
                elif new_command == "mvt":
                    twist_msg = Twist()
                    self.ang_target += 1
                    twist_msg.angular.z = self.ang_target
                    print(twist_msg)
                    self.state_publisher.publish(State.MAINTAIN_ANGLE)
                    self.target_publisher.publish(twist_msg)
                else:
                    print "Invalid command."

            else:
                print "Pressed this key: {}".format(self.key_pressed)
                if(self.key_pressed in key_actions.keys()):
                    # If the user command exists, find the velocity binding
                    action = key_actions[self.key_pressed]
                    polar_velocity = action_bindings[action]
                    twist_msg = Twist()
                    twist_msg.linear.x = polar_velocity.linear
                    twist_msg.angular.z = polar_velocity.angular
                    # Publish the velocity to the neato
                    self.cmd_vel_publisher.publish(twist_msg)
                    # Publish the current state to TELEOP, in case higher warmup_nav_arbiter is listening
                    self.state_publisher.publish(State.TELEOP)
                else:
                    print "Invalid key"

if __name__ == "__main__":
    # Print out teleop options
    print "Use the following keys to move the Neato around:"
    for k in key_actions.keys():
        print "'{}' : \t{}".format(k, key_actions[k])
    print "Type : and one of the following commands to call a behavior."
    for k in command_to_state.keys():
        print ":\t{}".format(k)

    node = TeleopNode()
    node.run()
