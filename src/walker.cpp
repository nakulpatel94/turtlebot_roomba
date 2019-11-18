/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Nakul Patel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file walker.cpp
 *
 * @author Nakul Patel
 *
 * @brief program to implement the walker functionality
 *
 * @version 1
 *
 * @date 2019-11-17
 *
 * @section DESCRIPTION
 *
 * Implementation of Walker class to enhance the functionality of turtlebot
 * walking(moving) with obstacle avoidance
 *
 */
#include <iostream>
#include "walker.hpp"

/**
 * @brief callback function for LaserScan
 * @param scan laser data published as sensor_msgs/LaserScan message
 * @return None
 *
 */
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& commandVel) {
  for (int i = 0; i < commandVel->ranges.size(); ++i) {
    /// Check if obstacle is present within distance specified
    if (commandVel->ranges[i] < 1.0) {
      obstacle = true;
      return;
    }
  }
  obstacle = false;
}


/**
 * @brief Implementation of Constructor.
 * @param None
 * @return None
 *
 */
Walker::Walker() {
  ROS_INFO_STREAM("Turtlebot_roomba intialized");

  /// Publish the velocity
  pubVel = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 1000);
  /// defining initial velocities in x and z directions
  commandVel.linear.x = 0.0;
  commandVel.linear.y = 0.0;
  commandVel.linear.z = 0.0;
  commandVel.angular.x = 0.0;
  commandVel.angular.y = 0.0;
  commandVel.angular.z = 0.0;

  /**
   * Subscribing to the laserScan data from /scan topic.
   * Also, call laserCallback to check for obstacles, if any and
   * set corresponding variables that can be used to move turtlebot
   * without hitting the obstacle.
   *
   */
  laserData = nh.subscribe < sensor_msgs::LaserScan
      > ("/scan", 500, &Walker::laserCallback, this);

  /// publishing velocities
  pubVel.publish(commandVel);
}


/**
 * @brief Implementation of Destructor.
 * @param None
 * @return None
 *
 */
Walker::~Walker() {
  /// Reset the turtlebot
  commandVel.linear.x = 0.0;
  commandVel.linear.y = 0.0;
  commandVel.linear.z = 0.0;
  commandVel.angular.x = 0.0;
  commandVel.angular.y = 0.0;
  commandVel.angular.z = 0.0;
  /// publish the reset velocities
  pubVel.publish(commandVel);
}

/**
 * @brief function to implement walking algorithm
 * @param None
 * @return None
 *
 */
void Walker::walk() {
  /// publish rate 10 Hz
  ros::Rate rate(10);

  while (ros::ok()) {
    if (obstacle) {
      ROS_WARN_STREAM("Obstacle has been detected.Rotating untill clear.");
      /// Stopping the robot in x direction
      commandVel.linear.x = 0.0;
      /// Rotating in z direction until the way is clear
      commandVel.angular.z = 0.2;
    } else {
      ROS_INFO_STREAM("Turtleboot_Roomba is walking.");
      /// Moving the robot in x direction
      commandVel.linear.x = 0.2;
      /// Setting zero rotation.
      commandVel.angular.z = 0.0;
    }

    /// publishng velocities
    pubVel.publish(commandVel);

    ros::spinOnce();

    /**
     * using the ros::Rate object to sleep for the time remaining to let us
     * hit our publish rate.
     */
    rate.sleep();
  }
}

