#!/usr/bin/env python
"""
Use potential field to avoid obstacles.
"""
import rospy
from sensor_msgs.msg import LaserScan
from warmup_project.msg import LabeledPolarVelocity2D as LabeledPolarVelocity2DMsg
from warmup_project.msg import PolarVelocity2D as PolarVelocity2DMsg  # Avoid ambiguity with the PolarVelocity2D class
from warmup_project.msg import State
from geometry_msgs.msg import Twist
import math

# TODO(matt): Move this out of this file
class PolarVelocity(object):
    def __init__(self, magnitude=0, angle=0, unit='rad', x=None, y=None):
        if x != None and y != None:
            self._x = x
            self._y = y
            return

        if unit in ('rad', 'radians'):
            angle = angle % (2 * math.pi)  # [rad]
        elif unit in ('deg', 'degree'):
            angle = math.radians(angle % 360)
        else:
            print("Unknown unit {}, assuming \'radians\'.".format(unit))
            angle = angle % (2 * math.pi)  # [rad]

        self._x = magnitude * math.cos(angle)
        self._y = magnitude * math.sin(angle)

    @property
    def magnitude(self):
        return math.sqrt(self._x**2 + self._y**2)

    @property
    def angle(self):
        return math.atan2(self._y, self._x)

    def __add__(self, other):
        return PolarVelocity(x=self._x + other._x, y=self._y + other._y)

    def __div__(self, scalar):
        # TODO(matt): Typecheck this
        return PolarVelocity(magnitude=self.magnitude / scalar, angle=self.angle)

    def __repr__(self):
        return "Magnitude: {} m/s\nAngle: {} degrees".format(self.magnitude, math.degrees(self.angle))

    def getMessage(self):
        """
        Generate PolarVelocity2DMsg for publishing to /desired_cmd_vel.
        """
        # Change the linear velocity so that it only moves forwards when the net_vel is in front of the neato
        magnitude = self.magnitude * (2 * (math.cos(self.angle)**2) - 1)
        print("magnitude: {}\nangle: {}".format(self.magnitude, math.degrees(self.angle)))
        print("cos(angle) = {}\n".format(math.cos(self.angle)))
        print("linear: {}\nangular: {}".format(magnitude, math.degrees(self.angle)))

        return PolarVelocity2DMsg(linear=magnitude, angular=self.angle)


class ObstacleAvoid(object):
    def __init__(self):
        behavior = "obstacle_avoid"
        rospy.init_node(behavior + "_node")

        self.behavior_id = State.OBSTACLE_AVOID

        # Setup publisher/subscription
        self.publisher_cmd_vel = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2DMsg, queue_size=10, latch=True)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.avoid_obstacle)

    def avoid_obstacle(self, msg):
        base_vel = PolarVelocity(1, 0)  # (magnitude, angle)
        obstacle_avoid_weight = 3  # strength of obstacle repelling

        # Do a running sum, then average the obstacle velocity component.
        net_obstacle_vel = PolarVelocity()
        range_count = 0
        for degree in range(360):
            degree_range = msg.ranges[degree]
            if degree_range != 0.0:
                range_count += 1
                obstacle_vel = PolarVelocity(
                    magnitude=obstacle_avoid_weight / ((degree_range/1.5) ** 2),
                    angle=degree + 180,
                    unit='degree')
                net_obstacle_vel += obstacle_vel

        if range_count == 0:
            print("No laser data")
            net_vel = base_vel
        else:
            net_obstacle_vel /= range_count
            net_vel = base_vel + net_obstacle_vel

        new_cmd_vel = LabeledPolarVelocity2DMsg(
            node_ID=self.behavior_id, 
            velocity=net_vel.getMessage())

        # print("Net obstacle velocity")
        # print(net_obstacle_vel)
        # print('')
        # print("Net velocity")
        # print(net_vel)
        print('\n----------------------')

        self.publisher_cmd_vel.publish(new_cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ObstacleAvoid()
    node.run()
