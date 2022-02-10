/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"
#include <std_msgs/Float64MultiArray.h>

namespace rotors_control
{

class LeePositionControllerNode
{
public:
	LeePositionControllerNode( const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
	~LeePositionControllerNode();

	void InitializeParams();
	void Publish();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	LeePositionController lee_position_controller_;

	std::string namespace_;

	// subscribers
	ros::Subscriber cmd_trajectory_sub_;
	ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
	ros::Subscriber cmd_sub_;
	ros::Subscriber odometry_sub_;
	ros::Subscriber iris1_control_input_sub_;
	ros::Subscriber iris2_control_input_sub_;

	ros::Publisher motor_velocity_reference_pub_;
	ros::Publisher position_error_pub_;
	ros::Publisher velocity_error_pub_;
	ros::Publisher attitude_error_pub_;
	ros::Publisher omega_error_pub_;
	ros::Publisher error_pub_;
	ros::Publisher iris1_original_rotor_vel_pub_;

	std_msgs::Float64MultiArray iris1_original_rotor_vel;

	mav_msgs::EigenTrajectoryPointDeque commands_;
	nav_msgs::OdometryConstPtr odometry_msg_;

	std::deque<ros::Duration> command_waiting_times_;
	ros::Timer command_timer_;

	void TimedCommandCallback(const ros::TimerEvent& e);
	/*
	void MultiDofJointTrajectoryCallback(
	        const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
	*/
	void CommandCallback(
	        const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& cmd_msg);

	void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

	void ControlInputCallback(const nav_msgs::OdometryConstPtr& control_input_msg);
	void Set_rotor_vel_multiarray(Eigen::Vector4d tmp);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
