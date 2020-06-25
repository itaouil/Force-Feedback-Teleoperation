/* inclusion guard */
#ifndef __UR10E_CONTROLLER_HPP__
#define __UR10E_CONTROLLER_HPP__

#include <utility>
#include <cmath>
#include <array>

// ROS imports
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/WrenchStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/collision_detection/collision_robot.h>

// imports for Eigen
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "../include/collision/collision_detection.hpp"


/*
 * main task:
 * 
 * - get pose from Franka Emika Panda
 * - transform pose to UR10e pose using IK
 * - measure wrench on UR10e
 * - feed data back to Franka Emika Panda
 */
class UR10eController {
private:
    trajectory_msgs::JointTrajectory performInternalIk(const Eigen::Isometry3d franka_pose);
    std::pair<bool, double> checkSelfCollision();
    bool validateTrajectorySpeed();
    void handleFrankaEEPose(const geometry_msgs::Pose::ConstPtr& msg);
    void performForceFeedback(Eigen::Isometry3d franka_pose);
    std::pair<bool, std::array<double,3>> closestPointBoundingBox(std::array<double,3> point);

    void handleUR10eTorqueForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);


    robot_model::RobotModelPtr m_robot_model;
    robot_state::RobotStatePtr m_robot_state;
    boost::shared_ptr<CollisionDetection> m_object_detect;
    planning_scene::PlanningScenePtr m_planning_scene;
    ros::NodeHandle m_node_handler;

    ros::Publisher m_ur10e_motion_pub;
    ros::Publisher m_franka_ff_pub; // to be removed, used only for bounding box
    ros::Publisher m_ur10e_wrench_pub;
    ros::Publisher m_ur10e_error_pub;
    ros::Subscriber m_ur10_wrench_sub;
    ros::Subscriber m_franka_pose_sub;

    Eigen::Isometry3d m_previous_franka_pose;

    trajectory_msgs::JointTrajectoryPoint m_tpoint_t;
    trajectory_msgs::JointTrajectoryPoint m_tpoint_t_minus_1;

    Eigen::Matrix<double, 6, 1> m_wrench_previous;

    
public:
   UR10eController();
   ~UR10eController();
};

#endif /* __UR10E_CONTROLLER_HPP__ */
