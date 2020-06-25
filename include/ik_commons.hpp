#ifndef IK_COMMONS_H
#define IK_COMMONS_H

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <string>

#include "../include/parameters.h"

trajectory_msgs::JointTrajectory performIk(const Eigen::Isometry3d&, const robot_model::RobotModelPtr&, robot_state::RobotStatePtr&, std::string);

#endif