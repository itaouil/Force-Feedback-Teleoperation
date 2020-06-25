#include "../include/ik_commons.hpp"

trajectory_msgs::JointTrajectory performIk(const Eigen::Isometry3d& goal_pose, const robot_model::RobotModelPtr& robot_model, robot_state::RobotStatePtr& robot_state, std::string robot_name)
{
    auto joint_group = robot_model->getJointModelGroup(robot_name);
    robot_state->setFromIK(joint_group, goal_pose, 0.0);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    auto joints = joint_group->getJointModels();
    for (auto &joint : joints)
    {
        if (!(joint->getType() == robot_model::JointModel::REVOLUTE || joint->getType() == robot_model::JointModel::PRISMATIC))
            continue; // only consider movable joints

        trajectory.joint_names.push_back(joint->getName());
        trajectory_point.positions.push_back(*(robot_state->getJointPositions(joint->getName())));
    }
    trajectory_point.time_from_start = ros::Duration(1 / F_MEASUREMENT_HERTZ);

    trajectory.points.push_back(trajectory_point);
    trajectory.header.stamp = ros::Time::now();

    return trajectory;
}