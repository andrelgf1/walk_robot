/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file walkRobot.hpp
 *
 * @brief Header file for declaration of class and  methods
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */

#ifndef INCLUDE_WALK_ROBOT_WALKROBOT_HPP_
#define INCLUDE_WALK_ROBOT_WALKROBOT_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class walkRobot {
 private:
/// obstacle variable that handles the presence of an obstacle
  bool obstacle;
/// variable with the number of obstacles avoided
  int avoided;

 public:
/**
 * @brief constructor of the walkRobot class
 * 
 * @param none
 * @return none
 */
  walkRobot();

/**
 * @brief Sensor callback function in order to subscribe in the laser sensor topic.
 * 
 * @param sensor_msgs::LaserScan
 * @return none
 */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr&);

/**
 * @brief Function checks if there are obstacles closer to the robot.
 * 
 * @param none
 * @return true or false, in case if there are or not obstacles close to the robot 
 */
  bool collisionCheck();

/**
 * @brief set number of obstacles avoided to variable avoided
 * 
 * @param number
 * @return none
 */
  void setAvoided(int number);

/**
 * @brief set number of obstacles avoided to variable avoided
 * 
 * @param none
 * @return int
 */
  int getAvoided();
};

#endif  // INCLUDE_WALK_ROBOT_WALKROBOT_HPP_"
