#!/usr/bin/env python
"""
This is a teleoperation script which allows you to control the robot, as well as change its state or mission. <- maybe this is better in another node?
"""
import rospy
from warmup_project.msg import DesiredVelocity, PolarVelocity2D, State
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Define mapping of command names to robot states
command_to_state = {
"estop" : State.ESTOP,
"wall": State.WALL_FOLLOW
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

        # TODO: to make this standalone, change the topic to be published to based on whether is launches standalone or not.
        self.cmd_vel_publisher = rospy.Publisher("/desired_cmd_vel", DesiredVelocity, queue_size=10)
        # self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.state_publisher = rospy.Publisher("/desired_state", State, queue_size=10)
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
            # TODO: may need a separate thread to concurrently keep publishing cmd_vel. Or set a timeout

            if(self.key_pressed == ':'):
                print(":"), # Print : on same line as input entry
                new_command = raw_input()
                # print "Entered command :{}".format(new_command)
                # Set state to new state if the state has changed.
                if(new_command in command_to_state.keys()):
                    self.state_publisher.publish(command_to_state[new_command])
                    print "Commanded state: {}".format(command_to_state[new_command])
                else:
                    print "Invalid command."

            else:
                # Send appropriate command.
                print "Pressed this key: {}".format(self.key_pressed)
                if(self.key_pressed in key_actions.keys()):
                    action = key_actions[self.key_pressed]
                    desired_vel = DesiredVelocity(node_ID=str(State.TELEOP), velocity=action_bindings[action])
                    print "Node state: {}\t Desired Vel: {}\t".format(State.TELEOP, desired_vel)
                    self.cmd_vel_publisher.publish(desired_vel)
                    # twist_msg = Twist()
                    # twist_msg.linear.x = desired_vel.velocity.linear
                    # twist_msg.angular.z = desired_vel.velocity.angular
                    # self.cmd_vel_publisher.publish(twist_msg)
                else:
                    print "Invalid key"

if __name__ == "__main__":
    node = TeleopNode()
    node.run()
