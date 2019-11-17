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
 * @brief main program to call the walker node
 *
 * @version 1
 *
 * @date 2019-11-17
 *
 * @section DESCRIPTION
 *
 *
 */
#include <iostream>
#include "walker.hpp"

/**
 * @brief Implementation of Constructor.
 * @param None
 * @return None
 */
Walker::Walker() {
  /// Publish the velocity
  pubVel = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 1000);
  /// defining initial velocities in x and z directions
  msg.linear.x = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.z = 0.0;
  /// stopping the publisher
  pubVel.publish(msg);

}

/**
 * @brief Implementation Destructor
 * @param None
 * @return None
 */
Walker::~Walker() {
  /// Reset the turtlebot
  msg.linear.x = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.z = 0.0;
  /// publish the reset velocities
  pubVel.publish(msg);
}

/**
 * @brief function to implement walking algorithm
 * @param None
 * @return None
 */
void Walker::walk() {
  /// publish rate 10 Hz
  ros::Rate loop(10);
 
  while (ros::ok()) {
 
    /// Moving the robot
    msg.linear.x = 1.0;
    /// Rotating the robot
    msg.angular.z = 0.0;

    /// publishng veloxities
    pubVel.publish(msg);

    ros::spinOnce();
    loop.sleep();
  }
}

