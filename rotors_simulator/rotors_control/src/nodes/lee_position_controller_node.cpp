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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"
#include <std_msgs/Float64MultiArray.h>

namespace rotors_control
{
LeePositionControllerNode::LeePositionControllerNode(const
                ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh),
	 private_nh_(private_nh)
{
	InitializeParams();

	cmd_sub_ = nh_.subscribe(
	                   "/iris1/desired_trajectory", 1,
	                   &LeePositionControllerNode::CommandCallback, this);

	iris1_control_input_sub_ = nh_.subscribe(
	                   "/iris1_control_input", 1,
	                   &LeePositionControllerNode::ControlInputCallback, this);				   

	odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
	                              &LeePositionControllerNode::OdometryCallback, this);

	motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
	                                        mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

	error_pub_ = nh_.advertise<nav_msgs::Odometry>("/iris1/error", 1);

	iris1_original_rotor_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/iris1_original_rotor_vel", 1);

	command_timer_ = nh_.createTimer(ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback, this,
	                                 true, false);
}

LeePositionControllerNode::~LeePositionControllerNode() { }

void LeePositionControllerNode::InitializeParams()
{
	// Read parameters from rosparam.
	GetRosParameter(private_nh_, "position_gain/x",
	                lee_position_controller_.controller_parameters_.position_gain_.x(),
	                &lee_position_controller_.controller_parameters_.position_gain_.x());
	GetRosParameter(private_nh_, "position_gain/y",
	                lee_position_controller_.controller_parameters_.position_gain_.y(),
	                &lee_position_controller_.controller_parameters_.position_gain_.y());
	GetRosParameter(private_nh_, "position_gain/z",
	                lee_position_controller_.controller_parameters_.position_gain_.z(),
	                &lee_position_controller_.controller_parameters_.position_gain_.z());
	GetRosParameter(private_nh_, "velocity_gain/x",
	                lee_position_controller_.controller_parameters_.velocity_gain_.x(),
	                &lee_position_controller_.controller_parameters_.velocity_gain_.x());
	GetRosParameter(private_nh_, "velocity_gain/y",
	                lee_position_controller_.controller_parameters_.velocity_gain_.y(),
	                &lee_position_controller_.controller_parameters_.velocity_gain_.y());
	GetRosParameter(private_nh_, "velocity_gain/z",
	                lee_position_controller_.controller_parameters_.velocity_gain_.z(),
	                &lee_position_controller_.controller_parameters_.velocity_gain_.z());
	GetRosParameter(private_nh_, "attitude_gain/x",
	                lee_position_controller_.controller_parameters_.attitude_gain_.x(),
	                &lee_position_controller_.controller_parameters_.attitude_gain_.x());
	GetRosParameter(private_nh_, "attitude_gain/y",
	                lee_position_controller_.controller_parameters_.attitude_gain_.y(),
	                &lee_position_controller_.controller_parameters_.attitude_gain_.y());
	GetRosParameter(private_nh_, "attitude_gain/z",
	                lee_position_controller_.controller_parameters_.attitude_gain_.z(),
	                &lee_position_controller_.controller_parameters_.attitude_gain_.z());
	GetRosParameter(private_nh_, "angular_rate_gain/x",
	                lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
	                &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
	GetRosParameter(private_nh_, "angular_rate_gain/y",
	                lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
	                &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
	GetRosParameter(private_nh_, "angular_rate_gain/z",
	                lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
	                &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
	GetVehicleParameters(private_nh_, &lee_position_controller_.vehicle_parameters_);
	lee_position_controller_.InitializeParameters();
}
void LeePositionControllerNode::Publish()
{
}

void LeePositionControllerNode::CommandCallback(
        const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& cmd_msg)
{
	// Clear all pending commands.
	command_timer_.stop();
	commands_.clear();
	command_waiting_times_.clear();

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	// put cmd_msg data into eigen_reference
	mav_msgs::eigenTrajectoryPointFromMsg(*cmd_msg, &eigen_reference);
	commands_.push_front(eigen_reference);
	// put eigen_reference into command_trajectory_ under the lee_position_controller_
	lee_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();
}

void LeePositionControllerNode::ControlInputCallback(
        const nav_msgs::OdometryConstPtr& control_input_msg)
{	

	// CalculateRotorVelocities() is called to calculate rotor velocities and put into ref_rotor_velocities
	Eigen::VectorXd ref_rotor_velocities;
	nav_msgs::Odometry error;
	lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &error, control_input_msg);

	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
	Set_rotor_vel_multiarray_1(ref_rotor_velocities);
	iris1_original_rotor_vel_pub_.publish(iris1_original_rotor_vel);

	actuator_msg->angular_velocities.clear();
	for (int i = 0; i < ref_rotor_velocities.size(); i++)
	{
		if(i==0 || i==2){
			actuator_msg->angular_velocities.push_back(sqrt(0.8)*ref_rotor_velocities[i]); //efficiency
		}else{
			actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]); //efficiency
		}
	}
	actuator_msg->header.stamp = odometry_msg_->header.stamp;

	motor_velocity_reference_pub_.publish(actuator_msg);
	error_pub_.publish(error);
}

void LeePositionControllerNode::Set_rotor_vel_multiarray_1(Eigen::Vector4d tmp){
	std::vector<double> vec1 = {tmp(0),tmp(1),tmp(2),tmp(3)};
	// copy in the data
	iris1_original_rotor_vel.data.clear();
	iris1_original_rotor_vel.data.insert(iris1_original_rotor_vel.data.end(), vec1.begin(), vec1.end());
}


void LeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e)
{
	if(commands_.empty()) {
		ROS_WARN("Commands empty, this should not happen here");
		return;
	}

	const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
	lee_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();
	command_timer_.stop();
	if(!command_waiting_times_.empty()) {
		command_timer_.setPeriod(command_waiting_times_.front());
		command_waiting_times_.pop_front();
		command_timer_.start();
	}
}

void LeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
	ROS_INFO_ONCE("LeePositionController got first odometry message.");

	// put sensor data from gazebo to lee_position_controller_
	// odometry contain position, orientation, velocity, angular_velocity
	EigenOdometry odometry;
	odometry_msg_ = odometry_msg;
	eigenOdometryFromMsg(odometry_msg, &odometry);
	lee_position_controller_.SetOdometry(odometry);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lee_position_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	rotors_control::LeePositionControllerNode lee_position_controller_node(nh, private_nh);

	// this node will call subscribe function as long as someone publish data to topic
	// LeePositionControllerNode::CommandCallback()
	// LeePositionControllerNode::MultiDofJointTrajectoryCallback()
	// LeePositionControllerNode::OdometryCallback()

	ros::spin();

	return 0;
}
