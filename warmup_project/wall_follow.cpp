#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

int followWall(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  scan_in.ranges
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wallFollowNode");

    ros::NodeHandle nodeHandle;

    ros::Subscriber laser_sub = nodeHandle.subscribe("/scan", 1000, followWall);

    ros::spin();

    return 0;
}