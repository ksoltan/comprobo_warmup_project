#!/usr/bin/env python
"""
Use potential field to avoid obstacles.
"""
import rospy
from sensor_msgs.msg import LaserScan
from warmup_project.msg import DesiredVelocity as DesiredVelocityMsg
from warmup_project.msg import PolarVelocity2D as PolarVelocity2DMsg  # Avoid ambiguity with the PolarVelocity2D class
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

    def getMessage(self):
        return PolarVelocity2DMsg(linear=self.magnitude, angular=self.angle)


class ObstacleAvoid(object):
    def __init__(self):
        behavior = "obstacle_avoid"
        rospy.init_node(behavior + "_node")

        self.behavior_id = behavior

        # Setup publisher/subscription
        self.publisher_cmd_vel = rospy.Publisher("/desired_cmd_vel", DesiredVelocityMsg, queue_size=10, latch=True)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.avoid_obstacle)

    def avoid_obstacle(self, msg):
        base_vel = PolarVelocity(1, 0)  # (magnitude, angle)
        obstacle_avoid_weight = 1  # strength of obstacle repelling

        # Do a running sum, then average the obstacle velocity component.
        net_obstacle_vel = PolarVelocity()
        range_count = 0
        for degree in range(360):
            degree_range = msg.range[degree]
            if degree_range != 0.0:
                range_count += 1
                obstacle_vel = PolarVelocity(
                    magnitude=obstacle_avoid_weight / (degree_range ** 2),
                    angle=degree + 180,
                    unit='degree')
                net_obstacle_vel += obstacle_vel
        net_obstacle_vel /= range_count
        
        net_vel = base_vel + net_obstacle_vel

        new_cmd_vel = DesiredVelocityMsg(behavior_ID=self.behavior_id, 
                desired_velocity=net_vel.getMessage())

        self.publisher_cmd_vel.publish(new_cmd_vel)
