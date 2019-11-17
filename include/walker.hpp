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
 * @file walker.hpp
 *
 * @author Nakul Patel
 *
 * @brief header file for class Walker
 *
 * @version 1
 *
 * @date 2019-11-17
 *
 * @section DESCRIPTION
 *
 * This file defines class Walker with some attributes and methods to
 * achieve walking behaviour for Turtlebot.
 */

#ifndef INCLUDE_WALKER_H_
#define INCLUDE_WALKER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


/**
 * @brief Walker Class.
 */
class Walker {
 private:
  /// variable for storing velocities
  geometry_msgs::Twist commandVel;
  /// node handler
  ros::NodeHandle nh;
  /// publish velocities
  ros::Publisher pubVel;
  /// subscribe to laserScan topic
  ros::Subscriber laserData;
  
  /**
   * to check if obstacle present,
   * default is false, set to true if obstacle detected
   */
  bool obstacle = false;

 public:
  /**
   * @brief constructor for walker object
   * @param None
   * @return None
   */
  Walker();
  /**
   * @brief destructor for walker object
   * @param None
   * @return None
   */
  ~Walker();

  /**
   * @brief function for moving robot
   * @param None
   * @return None
   */
  void walk();

  /**
   * @brief callback function for LaserScan
   * @param scan laser data published as sensor_msgs/LaserScan message
   * @return None
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& commandVel);
};

#endif  // INCLUDE_WALKER_HPP_

