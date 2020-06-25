#include "../../include/collision/collision_detection.hpp"

// minimum delay between calls to callback function
const ros::Duration CollisionDetection::m_min_delay(0.01);

CollisionDetection::CollisionDetection(
    robot_state::RobotStatePtr &robot_state,
    planning_scene::PlanningScenePtr &planning_scene,
    const std::string &marker_topic,
    const std::string &imarker_array_topic)
    : m_node_handler(),
      // create publishers for markers and robot state
      m_world_state_pub(m_node_handler.advertise<visualization_msgs::Marker>(marker_topic, 100)),
      m_marker_array_pub(m_node_handler.advertise<visualization_msgs::MarkerArray>(imarker_array_topic, 100)),
      m_franka_coll_pub(m_node_handler.advertise<force_feedback_controller::DoublePose>(TOPIC_DOUBLE_POSE, 1)),

      // load the robot description
      m_group(0), m_user_data(0), m_user_callback(0)
{
    m_robot_state = robot_state;
    m_planning_scene = planning_scene;
    m_robot_state->setToDefaultValues();
    m_group = m_robot_state->getJointModelGroup(UR10E_MODEL_GROUP);
    std::cout << m_group->getName() << std::endl;
    // Creation of geometry manager
    m_geometry_manager.reset(new GeometryManager(m_planning_scene, boost::bind(movedWorldMarkerCallback, this, _1)));
    m_geometry_manager->updateWorld();

    // start publishing timer
    m_init_time = ros::Time::now();
    m_last_callback_time = m_init_time;
    m_average_callback_duration = m_min_delay;
    m_schedule_request_count = 0;
    m_publish_timer = m_node_handler.createTimer(m_average_callback_duration, &CollisionDetection::updateCallback, this, true);

    // begin publishing robot state
    scheduleUpdate();
}

CollisionDetection::~CollisionDetection()
{
    delete m_group;
    m_geometry_manager.reset();
}

// callback called when marker moves.  Moves world object to new pose.
void CollisionDetection::movedWorldMarkerCallback(CollisionDetection *collision_detec,
                                               const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    Eigen::Isometry3d pose;
    tf2::fromMsg(feedback->pose, pose);
    collision_detec->updateObjectPose(pose);
    collision_detec->scheduleUpdate();
}

void CollisionDetection::updateObjectPose(const Eigen::Isometry3d &pose)
{
    m_geometry_manager->getGeometry()->updatePose(pose);
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void CollisionDetection::scheduleUpdate()
{
    // schedule an update callback for the future.
    // If the callback should run now, call it.
    if (setCallbackTimer(true))
        updateCallback(ros::TimerEvent());
}

/* callback called when it is time to publish */
void CollisionDetection::updateCallback(const ros::TimerEvent &e)
{
    ros::Time tbegin = ros::Time::now();
    m_publish_timer.stop();

    // #######################

    // Publish the geometry pose
    m_world_state_pub.publish(m_geometry_manager->getMarker());

    // Compute collisions with the arm
    computeCollisionContactPoints();

    // #######################

    // measure time spent in callback for rate limiting
    ros::Time tend = ros::Time::now();
    m_average_callback_duration = (m_average_callback_duration + (tend - tbegin)) * 0.5;
    m_last_callback_time = tend;
    m_schedule_request_count = 0;

    // schedule another callback if needed
    setCallbackTimer(false);
}

void CollisionDetection::computeCollisionContactPoints()
{
    m_geometry_manager->updateWorld();

    // creating a collision requests
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = m_group->getName();
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 6;
    c_req.verbose = false;

    // checking for collisions
    m_planning_scene->checkCollision(c_req, c_res, *m_robot_state);

    // Displaying Collision Contact Points
    if (c_res.collision && c_res.contact_count > 0)
    {
        // Elaborate and publish collisions
        elaborateCollision(c_res);
        publishCollision(c_res);
    }
    else
    {
        publishCollision(c_res);
    }
}

void CollisionDetection::elaborateCollision(collision_detection::CollisionResult &c_res)
{
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = c_res.contacts.begin(); it != c_res.contacts.end(); ++it)
    {
        std::vector<collision_detection::Contact> contacts;

        // keep only collisions with wrist_3_link
        if (it->first.second == "wrist_3_link")
        {
            contacts = it->second;
            // Extract end-effector pose collision
            collision_detection::Contact point_endeff = extractPointCollision(contacts, false);
            // Extract surface collision
            collision_detection::Contact point_surface = extractPointCollision(contacts, true);
            // From those two points it is possible to compute torques forces to
            // apply to the Franka arm in order to block the penetration of the object.
            contacts.clear();
            contacts.push_back(point_endeff);
            contacts.push_back(point_surface);
        }

        c_res.contacts[it->first] = contacts;
    }
}

collision_detection::Contact CollisionDetection::extractPointCollision(std::vector<collision_detection::Contact> &contacts, bool surface)
{
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    for (unsigned int i = 0; i < contacts.size(); i++)
    {
        if (surface)
            if(m_geometry_manager->getGeometry()->getType() == "sphere")
            // if the object is a sphere the collision detector return an inverse normal
                point += contacts[i].pos + contacts[i].normal * contacts[i].depth;
            else
                point += contacts[i].pos - contacts[i].normal * contacts[i].depth;
        else
            point += contacts[i].pos;
    }
    point /= contacts.size();
    collision_detection::Contact contact;
    contact.pos = point;
    return contact;
}

void CollisionDetection::publishCollision(collision_detection::CollisionResult &c_res)
{
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 0.7;

    if (c_res.collision && c_res.contact_count > 0)
    {
        /* Get the contact points and display them as markers */
        collision_detection::getCollisionMarkersFromContacts(markers, "base_link", c_res.contacts, color,
                                                             ros::Duration(), // remain until deleted
                                                             0.01);           // radius
    }

    // delete old markers
    if (m_collision_points.markers.size())
    {
        for (unsigned int i = 0; i < m_collision_points.markers.size(); i++)
            m_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

        m_marker_array_pub.publish(m_collision_points);
    }

    // move new markers into m_collision_points
    std::swap(m_collision_points.markers, markers.markers);

    // Move new markers into m_collision_points and
    // publish the feedback force for the franka arm.
    if (m_collision_points.markers.size())
    {
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = c_res.contacts.begin(); it != c_res.contacts.end(); ++it)
        {
            // Filter all the collision corresponding to the last link "wrist_3_link"
            if (it->first.second == "wrist_3_link")
            {
                ROS_INFO("COLLIDING with %s", it->first.first.c_str());
                force_feedback_controller::DoublePose double_msg;
                Eigen::Isometry3d pose_endeff(Eigen::Isometry3d(Eigen::Translation3d(it->second[0].pos)));
                Eigen::Isometry3d pose_surface(Eigen::Isometry3d(Eigen::Translation3d(it->second[1].pos)));
                geometry_msgs::Pose current_pose_msg;
                geometry_msgs::Pose target_pose_msg;

                tf::poseEigenToMsg(pose_endeff, current_pose_msg);
                tf::poseEigenToMsg(pose_surface, target_pose_msg);

                double_msg.surface_pose = target_pose_msg;
                double_msg.current_pose = current_pose_msg;

                // publish forcefeedback message
                m_franka_coll_pub.publish(double_msg);
                break;
            }
        }
        // publish markers position
        m_marker_array_pub.publish(m_collision_points);
    }
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool CollisionDetection::setCallbackTimer(bool new_update_request)
{
    m_publish_timer.stop();

    const ros::Time now = ros::Time::now();
    const ros::Duration desired_delay = std::max(m_min_delay, m_average_callback_duration * 1.2);
    ros::Duration sec_since_last_callback = now - m_last_callback_time;
    ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

    if (m_schedule_request_count)
    {
        // need a callback desired_delay seconds after previous callback
        m_schedule_request_count += new_update_request ? 1 : 0;
        if (sec_til_next_callback <= ros::Duration(0.0001))
        {
            // just run the callback now
            return true;
        }
        m_publish_timer.setPeriod(sec_til_next_callback);
        m_publish_timer.start();
        return false;
    }
    else if (new_update_request)
    {
        if (sec_til_next_callback < m_min_delay)
        {
            // been a while.  Use min_delay_.
            // Set last_callback_time_ to prevent firing too early
            sec_til_next_callback = m_min_delay;
            sec_since_last_callback = desired_delay - sec_til_next_callback;
            m_last_callback_time = now - sec_since_last_callback;
        }
        m_publish_timer.setPeriod(sec_til_next_callback);
        m_publish_timer.start();
        return false;
    }
    else if (!m_init_time.isZero())
    {
        // for the first few seconds after startup call the callback periodically
        // to ensure rviz gets the initial state.
        // Without this rviz does not show some state until markers are moved.
        if ((now - m_init_time).sec >= 8)
        {
            m_init_time = ros::Time(0, 0);
            return false;
        }
        else
        {
            m_publish_timer.setPeriod(std::max(ros::Duration(1.0), m_average_callback_duration * 2));
            m_publish_timer.start();
            return false;
        }
    }
    else
    {
        // nothing to do.  No callback needed.
        return false;
    }
}
