
#ifndef OBJECT_DETECTION_HPP
#define OBJECT_DETECTION_HPP

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection/collision_common.h>

#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <force_feedback_controller/DoublePose.h>

#include "imarker.hpp"
#include "geometry_manager.hpp"

/** Keeps track of the state of the robot and the world.
 * Updates the state when interactive markers are manipulated.
 * Publishes the state to rviz. */
class CollisionDetection
{
public:
    CollisionDetection(robot_state::RobotStatePtr &robot_state,
                    planning_scene::PlanningScenePtr &planning_scene,
                    const std::string &marker_topic = "interactive_robot_markers",
                    const std::string &imarker_array_topic = "interactive_robot_marray");
    ~CollisionDetection();

    void updateObjectPose(const Eigen::Isometry3d &pose);

    /* Indicate that the world or the robot has changed and
     * the new state needs to be updated and published to rviz */
    void scheduleUpdate();

    /** set a callback to call when updates occur */
    void setUserCallback(boost::function<void(CollisionDetection &robot)> callback)
    {
        m_user_callback = callback;
    }

    void *m_user_data;
private:
    /* callback called when marker moves.  Moves world object to new pose. */
    static void movedWorldMarkerCallback(CollisionDetection *object,
                                         const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    /* update the world and robot state and publish to rviz */
    void updateCallback(const ros::TimerEvent &e);

    /* functions to compute collisions and publish to rviz */
    void computeCollisionContactPoints();
    void elaborateCollision(collision_detection::CollisionResult &c_res);
    collision_detection::Contact extractPointCollision(std::vector<collision_detection::Contact> &contacts, bool surface);
    void publishCollision(collision_detection::CollisionResult &c_res);

    /* set the callback timer to fire if needed.
     * Return true if callback should happen immediately. */
    bool setCallbackTimer(bool new_update_request);

    /* marker publishers */
    ros::NodeHandle m_node_handler;
    ros::Publisher m_world_state_pub;
    ros::Publisher m_marker_array_pub;
    ros::Publisher m_franka_coll_pub;

    /* robot info */
    robot_state::RobotStatePtr m_robot_state;
    planning_scene::PlanningScenePtr m_planning_scene;
    shapes::ShapePtr m_world_cube_shape;
    visualization_msgs::MarkerArray m_collision_points;

    /* info about joint group we are manipulating */
    const robot_model::JointModelGroup *m_group;

    /* world info */
    std::shared_ptr<GeometryManager> m_geometry_manager;

    /* user callback function */
    boost::function<void(CollisionDetection &object)> m_user_callback;

    /* timer info for rate limiting */
    ros::Timer m_publish_timer;
    ros::Time m_init_time;
    ros::Time m_last_callback_time;
    ros::Duration m_average_callback_duration;
    static const ros::Duration m_min_delay;
    int m_schedule_request_count;
};

#endif // OBJECT_DETECTION_HPP
