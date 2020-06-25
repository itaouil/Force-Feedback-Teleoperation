#ifndef FRANKACONTROLLER_H
#define FRANKACONTROLLER_H

// General imports
#include <array>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <chrono>
#include <signal.h>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <boost/thread/thread.hpp>

// ROS imports
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/WrenchStamped.h"
#include <trajectory_msgs/JointTrajectory.h>
#include "eigen_conversions/eigen_msg.h"
#include <force_feedback_controller/DoublePose.h>
#include <force_feedback_controller/UnexpectedState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "moveit/robot_model_loader/robot_model_loader.h"

// Franka imports
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/control_types.h>

// Custom parameters
#include "../include/parameters.h"

#include "../include/ik_commons.hpp"

/**
 * Franka master node.
 */
class FrankaController
{
public:
    // Constructor 1
    explicit FrankaController(bool);

    // Constructor 2
    explicit FrankaController(const std::string &, franka::Robot &, franka::Model &, bool);

    // Destructor
    virtual ~FrankaController();

private:
    // Sets stiffness
    void inline setStiffness()
    {
        m_stiffness.setZero();
        m_stiffness.topLeftCorner(3, 3) << TRANSLATIONAL_STIFFNESS * Eigen::MatrixXd::Identity(3, 3);
        m_stiffness.bottomRightCorner(3, 3) << ROTATIONAL_STIFFNESS * Eigen::MatrixXd::Identity(3, 3);
    }

    // Sets damping
    void inline setDamping()
    {
        m_damping.setZero();
        m_damping.topLeftCorner(3, 3) << TRANSLATIONAL_STIFFNESS * Eigen::MatrixXd::Identity(3, 3);
        m_damping.bottomRightCorner(3, 3) << ROTATIONAL_STIFFNESS * Eigen::MatrixXd::Identity(3, 3);
    }

    // Get sign of a number
    int inline getSign(const double &val)
    {
        return ((0 < val) - (val < 0));
    }

    // Publishers
    void filePublisher();
    int robotPublisher(const franka::RobotState &);
    std::array<double, 16> extract_matrix(std::string &);
    void handleSimulatedEEPose(const geometry_msgs::Pose::ConstPtr &);

    // Collision behaviours
    void setDefaultBehavior();
    void setCollisionBehaviour();

    // Control loops
    void launchWrenchControl();
    void launchSimulatorControl();
    void launchDoublePoseControl();
    std::array<double, 7> computeDPTorque(Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian,
                                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis,
                                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq, 
                                    bool is_simulation = false);
    // Callbacks
    void writeToFile();
    void setWrenchStamped(const geometry_msgs::WrenchStamped::ConstPtr &);
    void setPoses(const force_feedback_controller::DoublePose::ConstPtr &);
    void handleUnexpectedState(const force_feedback_controller::UnexpectedState::ConstPtr &);

    // Control inner methods
    std::array<double, 7> filter_torques(std::array<double, 7>);
    Eigen::VectorXd limit_velocities(Eigen::Map<const Eigen::Matrix<double, 7, 1>>);

    // ROS members
    ros::NodeHandle m_n;
    ros::Rate m_loop_rate{F_MEASUREMENT_HERTZ};
    ros::Publisher m_ee_pose_pub;
    ros::Publisher m_ik_pub;
    ros::Publisher m_torque_pub;
    ros::Subscriber m_poses_sub;
    ros::Subscriber m_wrench_sub;
    ros::Subscriber m_ur10e_error_sub;
    ros::Subscriber m_interactive_marker_pose_sub;


    robot_model::RobotModelPtr m_moveit_robot_model;
    robot_state::RobotStatePtr m_moveit_robot_state;

    // Eigen members
    Eigen::VectorXd past_dq{7};
    Eigen::MatrixXd m_damping{6, 6};
    Eigen::MatrixXd m_stiffness{6, 6};
    Eigen::Isometry3d m_surface_pose;
    Eigen::Isometry3d m_current_pose;
    Eigen::Matrix<double, 6, 1> m_wrench;

    // Franka members
    bool m_first_loop = true;
    franka::RobotState m_robot_state;
    std::array<double, 7> m_tau_d_array;
    std::array<double, 7> m_prev_torques{0};
    boost::shared_ptr<franka::Model> m_model;
    boost::shared_ptr<franka::Robot> m_robot;
    std::array<double, 7> m_limit_torques{20, 20, 20, 18, 3, 3, 5};

    // ROS messages
    geometry_msgs::Pose m_pose_msg;

    // Params for writing to file
    std::thread m_thd_wtf;
    std::mutex m_mtx_wtf;
    bool m_write_to_file = false;
};

#endif // FRANKACONTROLLER_H
