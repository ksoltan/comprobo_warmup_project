"""
Makes Neato move parallel to a wall.
"""

from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians
import rospy

class WallFollow(object):
    def __init__(self):
        rospy.init_node("wall_follow_node")

        # Setup publisher/subscription
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.follow_wall)

    def follow_wall(self, scan_msg):
        max_side_angle = 65
        baseline_angular_speed = 1
        top_angular_speed = 2
        linear_speed = 1
        desired_distance_to_wall = 1
        k_p = 10
        # TODO: clamp the error: if it is below like 0.02, it's probably parallel enough
        distance_estimate = self.estimate_distance_to_wall(scan_msg.ranges, max_side_angle)
        angle_error = self.get_angular_error(scan_msg.ranges, max_side_angle)

        if distance_estimate is None:
            distance_error = 0
        else:
            distance_error = distance_estimate - desired_distance_to_wall
            print("distance_error = {}".format(distance_error))

        if(angle_error != None):
            error = angle_error + distance_error

            # publish a new cmd_vel
            angular_speed = min(k_p * error * baseline_angular_speed, top_angular_speed)
            new_cmd_vel = Twist(linear=Vector3(linear_speed, 0, 0), angular=Vector3(0, 0, angular_speed))
            print("New angular vel = {}".format(k_p * error * baseline_angular_speed))
            self.publisher_cmd_vel.publish(new_cmd_vel)

        print()


    def get_angular_error(self, ranges, max_side_angle):
        running_sum = 0
        range_count = 0
        for angle in range(1, max_side_angle):
            # TODO: error check so that we don't go over 180 degrees. Might be fine with wrapping?
            front_range = ranges[90 - angle]
            back_range = ranges[90 + angle]

            if front_range != 0.0 and back_range != 0.0:
                running_sum += front_range - back_range
                range_count += 1.0

        if range_count == 0.0:
            print("ah fuck")
            return None
        else:
            error = running_sum / range_count
            print("angle error = {}".format(error))
            return error

    def estimate_distance_to_wall(self, ranges, max_side_angle):
        distance_sum = 0
        distance_count = 0
        for angle in range(1, max_side_angle):
            front_range = ranges[90 - angle]
            back_range = ranges[90 + angle]

            if front_range != 0.0 and back_range != 0.0:
                area_cross_product = front_range * back_range * sin(radians(angle) * 2)
                third_length = sqrt(front_range ** 2 + back_range ** 2 - 2.0 * front_range * back_range * cos(radians(angle) * 2))
                wall_distance = 2.0 * area_cross_product / third_length

                distance_sum += wall_distance
                distance_count += 1.0

        if distance_count == 0:
            return None
        else:
            print("Wall distance estimate = {}".format(distance_sum / distance_count))
            return distance_sum / distance_count

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = WallFollow()
    node.run()