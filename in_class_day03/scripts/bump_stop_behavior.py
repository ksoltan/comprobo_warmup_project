""" This program controls the Neato to move forward at a fixed speed
and stop when it bumps into something."""
from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
import rospy

class BumpStopBehavior(object):
    def __init__(self):
        rospy.init_node("wary_neeto")
        self.my_cmd_vel = Twist(linear = Vector3(1, 0, 0), angular = Vector3(0, 0, 0))
        print(self.my_cmd_vel)
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.subscriber_bump = rospy.Subscriber("/bump", Bump, self.emergency_stop)

    def emergency_stop(self, bump_msg):
        # If any of the bump sensors are depressed, STAHP.
        if(bump_msg.leftSide or bump_msg.leftFront or bump_msg.rightSide or bump_msg.rightFront):
            print("EMERGENCY. STAHHHHP")
            self.my_cmd_vel.linear = Vector3(0, 0, 0)
            self.my_cmd_vel.angular = Vector3(0, 0, 0)

            self.publisher_cmd_vel.publish(self.my_cmd_vel)

    def run(self):
        # RUN ZE ROBIT!
        print("Woah. We started it. It should be moving. Plz")
        self.publisher_cmd_vel.publish(self.my_cmd_vel) # publish fixed speed forward
        rospy.spin()

if __name__ == "__main__":
    node = BumpStopBehavior()
    node.run()
