""" This program controls the Neato to move forward at a fixed speed
and steer away from obstacles. With lasers."""
from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy

def mean(values):
    if(len(values) == 0):
        return 0
    return sum(values) / len(values)

class PewPewNode(object):
    def __init__(self):
        rospy.init_node("pewpew_node")
        print("Initialized node")
        # Setup publisher/subscription
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.steer_away)

    def steer_away(self, scan_msg):
        angle = int(30)
        worry_distance = 2  # [m] start steering steer_away
        panic_distance = 0.5 # [m] pure, unstoppable, rotation
        top_speed = 1  # [m]
        angular_speed = 1

        # Make distance estimate
        # print("len(ranges) = {}".format(len(scan_msg.ranges)))
        front_range = scan_msg.ranges[:angle/2] + scan_msg.ranges[-1 * angle / 2:]
        # print("front_range = {}".format(front_range))
        valid_readings = self.filter_readings(front_range, scan_msg)
        if(not valid_readings):
            # If no good readings, robot shouldn't move! Or should continue in whatever fashion it was previously
            return

        # print("valid_readings = {}".format(valid_readings))
        distance_estimate = mean(valid_readings)
        # print("distance_estimate = {}".format(distance_estimate))

        turn_dir = self.choose_turn_dir(scan_msg, angle)
        print("turn_dir = {}".format(turn_dir))
        # Determine vel command
        if distance_estimate > worry_distance:
            cmd_vel = Twist(linear = Vector3(1, 0, 0), angular = Vector3(0, 0, 0))

        elif distance_estimate > panic_distance:
            print('"I don\'t know about this..." -neato probably')
            alpha = (distance_estimate - panic_distance) / (worry_distance - panic_distance)
            speed = top_speed * alpha
            cmd_vel = Twist(linear = Vector3(speed, 0, 0), angular = Vector3(0, 0, turn_dir * angular_speed))

        else:
            print("PANIC PANIC PANIC")
            cmd_vel = Twist(linear = Vector3(0, 0, 0), angular = Vector3(0, 0, turn_dir * angular_speed))

        # Publish
        self.publisher_cmd_vel.publish(cmd_vel) # publish fixed speed forward

    def choose_turn_dir(self, scan_msg, front_angle):
        #  Average 45 degrees of data to left of front range and to right of front ranges
        side_angle = 45
        turning_distance_threshold = 0.1

        if front_angle / 2 > side_angle:
            print("front angle larger than side angle: {} > {}".format(front_angle / 2, side_angle))
            return -1

        left_range = self.filter_readings(scan_msg.ranges[front_angle / 2 : side_angle], scan_msg)
        # print("left_range = {}".format(left_range))
        right_range = self.filter_readings(scan_msg.ranges[-side_angle : -front_angle / 2], scan_msg)
        # print("right_range = {}".format(right_range))
        # compare the left and front, choose side with farther obstacle
        left_estimate = mean(left_range)
        right_estimate = mean(right_range)

        if left_estimate == 0.0:
            print("Invalid readings on the left, turning right")
            return -1
        elif right_estimate == 0.0:
            print("Invalid readings on the right, turning left")
            return 1

        # Avoid oscillation between turning left and right by adding a threshold
        # by making it harder to decide to go right
        if(left_estimate + turning_distance_threshold < right_estimate):
            # Want to go right
            return -1
        return 1 # Go left

    def filter_readings(self, readings, scan_msg):
        return [reading for reading in readings if self.is_valid_reading(reading, scan_msg)]

    def is_valid_reading(self, reading, scan_msg):
        return scan_msg.range_min <= reading <= scan_msg.range_max

    def pew(self):
        rospy.spin()

if __name__ == "__main__":
    pew = PewPewNode()
    pew.pew()
