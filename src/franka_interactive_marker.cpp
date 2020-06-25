// ROS related
#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

// General
#include <math.h>

#include "../include/parameters.h"

using namespace visualization_msgs;

// Global vars
ros::Publisher pub;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_DEBUG_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  // Publish pose feedback message
  pub.publish(feedback->pose);
}

/**
 * Create interactive marker.
 */
Marker makeBox(InteractiveMarker &msg)
{
  // Marker object
  Marker marker;
  
  // Marker settings:
  //  - Shape
  //  - Scale (in meters)
  //  - Color (including alpha)
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.5;

  return marker;
}

/**
 * Create a non-interactive control
 * which contains the box
 */
InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg)
{
  // Marker control object
  InteractiveMarkerControl control;

  // Always visibility in RVIZ
  control.always_visible = true;

  // Add our cube marker to the control marker
  control.markers.push_back( makeBox(msg) );

  // Add the control to the interactive marker
  msg.controls.push_back( control );

  return msg.controls.back();
}

/**
 * Transforms magic.
 */
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}

/**
 * 6 DoF marker.
 */
void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof)
{
  // Create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.5;
  int_marker.name = "franka_ee_6dof";
  int_marker.description = "Franka-EE 6-DOF Control";

  // Insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;
  
  // In case we want to
  // fix the cube marker
  // orientation
  if (fixed)
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
      {
        mode_text = "MOVE_3D";
      }

      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
      {
        mode_text = "ROTATE_3D";
      }

      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
      {
        mode_text = "MOVE_ROTATE_3D";
      }
      
      int_marker.name += "_" + mode_text;

      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

int main(int argc, char** argv)
{
  // Creat ROS node
  ros::init(argc, argv, "franka_interactive_marker");
  ros::NodeHandle n;

  // Create 6DoF publisher
  pub = n.advertise<geometry_msgs::Pose>(TOPIC_FRANKA_EE_POSE, 1);

  // Create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer(TOPIC_FRANKA_EE_POSE,"",false));

  ros::Duration(0.1).sleep();

  // Franka endpose effector 6DoF
  make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, INIT_FRANKA_POS, true);
  geometry_msgs::Pose pose;
  pose.position.x = INIT_FRANKA_POS.getX();
  pose.position.y = INIT_FRANKA_POS.getY();
  pose.position.z = INIT_FRANKA_POS.getZ();
  pose.orientation.w = 1.0;
  pose.orientation.x = 3.18*2;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  server->setPose("franka_ee_6dof", pose);

  server->applyChanges();

  ros::spin();
  
  server.reset();
}
