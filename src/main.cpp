/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file main.cpp
 *
 * @brief Node cantrols turtlebot in order to move without hitting obstacles
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "walk_robot/walkRobot.hpp"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "walkRobot");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

/// creating a object of the class walkRobot
  walkRobot robot;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  auto sensorLaser = n.subscribe<sensor_msgs::LaserScan>("/scan", 50,
&walkRobot::laserSensorCallback, &robot);

   /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  auto velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

/// creating velocity object variable
  geometry_msgs::Twist msg;

/// Set loop rate according to desired frequency
  ros::Rate loop_rate(10);

/// count variable to count number of obstacles avoided
  int count = 0;

/// variable used to identify if it is a new object or it is the same obstacle
  bool flag = false;

  while (ros::ok()) {
/// check obstacle variable in order to ser the velocity
    if ( robot.collisionCheck() == false ) {
      flag = true;
/// Set linear velocity in the x direction to 1
      msg.linear.x = 1.0;
      ROS_INFO_STREAM("Setting linear velocity");
      ROS_INFO_STREAM("Number of obstacles avoided "<< robot.getAvoided());
/// Make sure that the robot does not have any angular velocity
      msg.angular.z = 0.0;
      } else {
/// if the robot detects an obstacle set angular velocity
      msg.angular.z = 0.4;
      ROS_INFO_STREAM("Setting angular velocity to Avoid Obstacle.");
/// Make sure the robot does not have any linear velocity
      msg.linear.x = 0.0;
/// Increment the number of obstacles only if the flag equal true
      if ( flag == true ) {
        ++count;
        robot.setAvoided(count);
        flag = false;
      }
    }
/// publishing the velocity
    velocity.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

