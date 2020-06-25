#include "../include/ur10e_controller.hpp"
#include "../include/ik_commons.hpp"
#include "../include/parameters.h"
#include <force_feedback_controller/DoublePose.h>
#include <force_feedback_controller/UnexpectedState.h>

#include "../include/collision/collision_detection.hpp"

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>

UR10eController::UR10eController()
    : m_node_handler("~")
{
    ROS_INFO("Constructing UR10eController");
    boost::shared_ptr<robot_model_loader::RobotModelLoader> moveit_loader;
    moveit_loader = boost::make_shared<robot_model_loader::RobotModelLoader>(
        "robot_description");
    m_robot_model = moveit_loader->getModel();
    m_robot_state.reset(new robot_state::RobotState(m_robot_model));
    m_planning_scene.reset(new planning_scene::PlanningScene(m_robot_model));

    m_wrench_previous.setConstant(0);

    m_ur10e_motion_pub = m_node_handler.advertise<trajectory_msgs::JointTrajectory>(TOPIC_UR10e_JOINT_TRAJECTORY, 1);
    m_franka_ff_pub = m_node_handler.advertise<force_feedback_controller::DoublePose>(TOPIC_DOUBLE_POSE, 1);
    m_ur10e_error_pub = m_node_handler.advertise<force_feedback_controller::UnexpectedState>(TOPIC_UR10e_UNEXP_STATE, 1);
    m_ur10e_wrench_pub = m_node_handler.advertise<geometry_msgs::WrenchStamped>(TOPIC_UR10e_FF, 1);
    m_franka_pose_sub = m_node_handler.subscribe<geometry_msgs::Pose>(TOPIC_FRANKA_EE_POSE, 1, &UR10eController::handleFrankaEEPose, this);
    m_ur10_wrench_sub = m_node_handler.subscribe<geometry_msgs::WrenchStamped>(TOPIC_UR10e_WRENCH, 1, &UR10eController::handleUR10eTorqueForce, this);

    // CollisionDetection initialization
    m_object_detect.reset(new CollisionDetection(m_robot_state, m_planning_scene));
}

UR10eController::~UR10eController()
{
    m_robot_state.reset();
    m_planning_scene.reset();
}

void UR10eController::handleFrankaEEPose(const geometry_msgs::Pose::ConstPtr &msg)
{
    Eigen::Isometry3d franka_pose_received;
    tf::poseMsgToEigen(*msg, franka_pose_received);
    trajectory_msgs::JointTrajectory calculated_trajectory = performInternalIk(franka_pose_received);
    if (!validateTrajectorySpeed())
    {
        ROS_ERROR("Exiting due to too high speed of joint");
        force_feedback_controller::UnexpectedState message;
        message.unexpectedState = ERROR_V_MAX_EXCEEDED;
        m_ur10e_error_pub.publish(message);
        // exit(1);
    }
    std::pair<bool, double> collision_res = checkSelfCollision();
    if (collision_res.first)
    {
        ROS_ERROR("Self collision detected! exiting!");
        force_feedback_controller::UnexpectedState message;
        message.unexpectedState = ERROR_SELF_COLLISION;
        m_ur10e_error_pub.publish(message);
        // exit(1);
    }
    else
    {
        ROS_DEBUG("Distance to self collision: %f", collision_res.second);
    }

    // if we passed until here, everything is fine and the UR10e may carry out the action
    m_ur10e_motion_pub.publish(calculated_trajectory);
    m_object_detect->scheduleUpdate();
    performForceFeedback(franka_pose_received);
}

void UR10eController::performForceFeedback(Eigen::Isometry3d franka_pose)
{
    force_feedback_controller::DoublePose double_msg;

    std::array<double, 3> pose_array;
    for (unsigned int i = 0; i < 3; i++)
    {
        pose_array[i] = franka_pose.translation().data()[i];
    }
    std::pair<bool, std::array<double, 3>> closest_point = closestPointBoundingBox(pose_array);

    if (!closest_point.first)
    {
        ROS_INFO("Left bounding");
        // we are outside the bounding box
        geometry_msgs::Pose surface_pose_msg;
        geometry_msgs::Pose current_pose_msg;

        // TODO previous rotation...might also use current rotation -> evaluate
        Eigen::Isometry3d pose_to_return_to(franka_pose);
        double *pose_array = pose_to_return_to.data();
        for (int i = 0; i < 3; i++)
        {
            pose_array[12 + i] = closest_point.second[i];
        }

        tf::poseEigenToMsg(pose_to_return_to, surface_pose_msg);
        tf::poseEigenToMsg(franka_pose, current_pose_msg);

        double_msg.surface_pose = surface_pose_msg;
        double_msg.current_pose = current_pose_msg;
    }
    else
    {
        // we are inside the bounding box, save current pose for going back here if we leave the bounding box in the next iteration
        m_previous_franka_pose = franka_pose;
        geometry_msgs::Pose surface_pose_msg;
        geometry_msgs::Pose current_pose_msg;

        tf::poseEigenToMsg(franka_pose, surface_pose_msg);
        tf::poseEigenToMsg(franka_pose, current_pose_msg);

        // Populate custom message
        double_msg.surface_pose = surface_pose_msg;
        double_msg.current_pose = current_pose_msg;
    }

    // Publish message
    m_franka_ff_pub.publish(double_msg);
}

std::pair<bool, double> UR10eController::checkSelfCollision()
{
    collision_detection::CollisionRobotPtr col_rob = m_planning_scene->getCollisionRobotNonConst();

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.max_contacts = 1000; // reduce computational effort
    col_req.distance = true;     // enable calculation of distance
    m_planning_scene->setCurrentState(*m_robot_state);
    col_rob->checkSelfCollision(col_req, col_res, m_planning_scene->getCurrentStateNonConst(), m_planning_scene->getAllowedCollisionMatrix());

    if (col_res.distance == 0)
    {
        ROS_INFO("Collision detected in IK");
        return std::pair<bool, double>(true, 0);
    }
    else
    {
        return std::pair<bool, double>(false, col_res.distance);
    }
}

bool UR10eController::validateTrajectorySpeed()
{
    if (m_tpoint_t_minus_1.positions.empty())
        return true; // inital new pose cannot be compared to previous

    double max_speed = 0;
    for (unsigned int i = 0; i < m_tpoint_t.positions.size(); i++)
    {
        double timestep = 1 / F_MEASUREMENT_HERTZ;
        double delta_joint_position = std::abs(m_tpoint_t_minus_1.positions[i] - m_tpoint_t.positions[i]);

        double speed = delta_joint_position / timestep;
        max_speed = std::max(max_speed, speed);
    }
    if (max_speed > V_MAX_JOINT)
    {
        ROS_INFO("Joint speed out of bounds!");
    }
    return max_speed <= V_MAX_JOINT;
}

trajectory_msgs::JointTrajectory UR10eController::performInternalIk(const Eigen::Isometry3d franka_pose)
{
    // rotation is needed to align end-effector of Franka arm with UR10e
    std::array<double, 16> rotation_offset_ur10e = {
        1, 0, 0, 0,
        0, 0, -1, -2,
        0, 1, 0, 0,
        0, 0, 0, 1};
    Eigen::Isometry3d ur10e_goal_pose; //  must not be combined with assignment due to type conversions
    ur10e_goal_pose = franka_pose * Eigen::Matrix4d::Map(rotation_offset_ur10e.data()).transpose();

    trajectory_msgs::JointTrajectory trajectory = performIk(ur10e_goal_pose, m_robot_model, m_robot_state, UR10E_MODEL_GROUP);

    // update saved trajectory points
    m_tpoint_t_minus_1 = m_tpoint_t;
    m_tpoint_t = trajectory.points[0];

    return trajectory;
}

void UR10eController::handleUR10eTorqueForce(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    Eigen::Matrix<double, 6, 1> wrench_unpacked;
    tf::wrenchMsgToEigen(msg->wrench, wrench_unpacked);

    for (unsigned int i = 0; i < wrench_unpacked.size(); i++)
    {
        wrench_unpacked.data()[i] = (1 - A_LOWPASS) * m_wrench_previous[i] + A_LOWPASS * (-1) * wrench_unpacked.data()[i];
    }
    m_wrench_previous = wrench_unpacked;
    geometry_msgs::WrenchStamped filtered_wrench_stamped;
    tf::wrenchEigenToMsg(wrench_unpacked, filtered_wrench_stamped.wrench);
    filtered_wrench_stamped.header.stamp = ros::Time::now();
    m_ur10e_wrench_pub.publish(filtered_wrench_stamped);
}

std::pair<bool, std::array<double, 3>> UR10eController::closestPointBoundingBox(std::array<double, 3> point)
{
    std::array<double, 3> closest_point;
    bool inside = true;
    for (unsigned int i = 0; i < 3; i++)
    {

        if (BOUNDING_BOX[i][0] <= point[i] && point[i] <= BOUNDING_BOX[i][1])
        {
            // we are inside the bounding box on axis i
            closest_point[i] = point[i];
        }
        else if (BOUNDING_BOX[i][0] > point[i])
        {
            // we are below the lower end of the bounding box on axis i
            closest_point[i] = BOUNDING_BOX[i][0];
            inside = false;
        }
        else
        {
            // we are above the upper end of the bounding box on axis i
            closest_point[i] = BOUNDING_BOX[i][1];
            inside = false;
        }
    }
    return std::pair<bool, std::array<double, 3>>(inside, closest_point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur10e_controller");
    ros::NodeHandle nh;
    UR10eController ur10e_controller;

    ros::spin();
    return 0;
}
