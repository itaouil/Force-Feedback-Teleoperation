#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <eigen3/Eigen/Core>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vector>

const std::string PACKAGE_NAME = "force_feedback_controller";

const float TIME_STAMP = 0.001; // freq. 1 kHz

const float F_MEASUREMENT_HERTZ = 2.0;
const float V_MAX_JOINT = 1.0; // max. velocity of joints in rad/s

const double A_LOWPASS = 0.1;

// The bounding reflect the size of the table in the x and y direction
// const double BOUNDING_BOX[3][2] = {
//     {-0.3, 0.73}, // x (min, max)
//     {-0.6, 0.1}, // y (min, max)
//     {0.0, 0.75}  // x (min, max)
// };
// Standard bounding box
const double BOUNDING_BOX[3][2] = {
    {-1, 1}, // x (min, max)
    {-1, 1}, // y (min, max)
    {-1, 1}  // x (min, max)
};

// Feedback node
const float ROTATIONAL_STIFFNESS{10.0};
const float TRANSLATIONAL_STIFFNESS{200.0};

const unsigned int ERROR_V_MAX_EXCEEDED = 1;
const unsigned int ERROR_SELF_COLLISION = 2;

const float TORQUE_LIMIT = 0;

// Velocity compensation
const float MAX_VELOCITY4 = 2.0;
const float MAX_VELOCITY3 = 2.4;

const tf::Vector3 INIT_FRANKA_POS(0, -2, 1);
const Eigen::Vector3d INIT_FRANKA_BASE_POSE(0, -2, 0);

const std::string TOPIC_UR10e_JOINT_TRAJECTORY = "/dummy/arm_controller/command";
const std::string TOPIC_UR10e_UNEXP_STATE = "/ur10e/unexpectedState";
const std::string TOPIC_UR10e_FF = "/ur10e/force_feedback";
const std::string TOPIC_UR10e_WRENCH = "/opto_ros/forces";

const std::string TOPIC_FRANKA_EE_POSE = "/franka/end_effector_pose";
const std::string TOPIC_FRANKA_TORQUE = "/franka/torque_forces";
const std::string TOPIC_FRANKA_JOINT_TRAJECTORY = "/dummy/arm_controller/command";

const std::string TOPIC_DOUBLE_POSE = "/collision/double_pose";

const std::string UR10E_MODEL_GROUP = "ur10e";
const std::string FRANKA_MODEL_GROUP = "franka_panda";

#endif