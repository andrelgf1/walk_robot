/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file walkRobot.cpp
 *
 * @brief Implementation of methods of the walkRobot class
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walk_robot/walkRobot.hpp"

walkRobot::walkRobot() {
/// initializing obstacle variable to false
  obstacle = false;
/// Initialize avoided variable to 0
  avoided = 0;
}

void walkRobot::laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
/// checking if laser found any obstacle
  for (auto i : msg->ranges) {
    if (i < 1) {
      obstacle = true;
      return;
    }
  }
/// if no obstacle was found set obstacle to false
  obstacle = false;
}

bool walkRobot:: collisionCheck() {
  return obstacle;
}

void walkRobot::setAvoided(int number) {
/// set number to variable avoided
  avoided = number;
  ROS_INFO_STREAM("Number of obstacles avoided"<< avoided);
}
/// get value of variable avoided
int walkRobot::getAvoided() {
  return avoided;
}



