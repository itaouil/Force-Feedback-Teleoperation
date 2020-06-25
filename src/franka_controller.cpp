/**
 * Franka Controller Class.
 * 
 * The aim of this class is to handle
 * all communications with the Franka
 * robotic arm. This includes accessing
 * information about the robot state, such
 * as end effector pose, torques, or anything
 * else, as well as send commands to the arm
 * directly.
 */

// Header file
#include "../include/franka_controller.hpp"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>

bool run_status = true;
struct sigaction old_action;

void quitHandler(int dummy)
{
  std::cout << "exit program." << std::endl;
  sigaction(SIGINT, &old_action, NULL);
  run_status = false;
}

/**
 * Constructor 1
 */
FrankaController::FrankaController(bool interactive_simulated_franka)
{
  m_ur10e_error_sub = m_n.subscribe<force_feedback_controller::UnexpectedState>(TOPIC_UR10e_UNEXP_STATE, 1, &FrankaController::handleUnexpectedState, this);

  if (!interactive_simulated_franka)
  {
    // Initialize publisher
    m_ee_pose_pub = m_n.advertise<geometry_msgs::Pose>(TOPIC_FRANKA_EE_POSE, 1);

    // Launch file pubsliher
    filePublisher();
  }
  else
  {
    // Set moveit and IK
    m_poses_sub = m_n.subscribe<force_feedback_controller::DoublePose>(TOPIC_DOUBLE_POSE, 1, &FrankaController::setPoses, this);
    m_interactive_marker_pose_sub = m_n.subscribe<geometry_msgs::Pose>(TOPIC_FRANKA_EE_POSE, 1, &FrankaController::handleSimulatedEEPose, this);
    boost::shared_ptr<robot_model_loader::RobotModelLoader> moveit_loader;
    moveit_loader = boost::make_shared<robot_model_loader::RobotModelLoader>(
        "robot_description");
    m_moveit_robot_model = moveit_loader->getModel();
    m_moveit_robot_state.reset(new robot_state::RobotState(m_moveit_robot_model));
    m_ik_pub = m_n.advertise<trajectory_msgs::JointTrajectory>(TOPIC_FRANKA_JOINT_TRAJECTORY, 1);
    m_torque_pub = m_n.advertise<std_msgs::Float64MultiArray>(TOPIC_FRANKA_TORQUE, 1);

    // Launch simulator control
    launchSimulatorControl();
  }
}

/**
 * Constructor 2
 */
FrankaController::FrankaController(const std::string &address, franka::Robot &robot, franka::Model &model, bool write_to_file = false) : m_n{"~"}
{
  // Reset m_robot instance
  m_robot.reset(&robot);

  // Reset m_model instance
  m_model.reset(&model);

  // Initialize publisher
  m_ee_pose_pub = m_n.advertise<geometry_msgs::Pose>(TOPIC_FRANKA_EE_POSE, 1);

  // Initialize subsribers
  m_wrench_sub = m_n.subscribe<geometry_msgs::WrenchStamped>(TOPIC_UR10e_FF, 1, &FrankaController::setWrenchStamped, this);
  m_poses_sub = m_n.subscribe<force_feedback_controller::DoublePose>(TOPIC_DOUBLE_POSE, 1, &FrankaController::setPoses, this);
  m_ur10e_error_sub = m_n.subscribe<force_feedback_controller::UnexpectedState>(TOPIC_UR10e_UNEXP_STATE, 1, &FrankaController::handleUnexpectedState, this);

  // Set compliance parameters
  setDamping();
  setStiffness();

  // Set default behaviour
  setDefaultBehavior();

  // Set collision behaviour
  setCollisionBehaviour();

  // Writing Robot state to file
  m_write_to_file = write_to_file;
  if (m_write_to_file)
    m_thd_wtf = std::thread(&FrankaController::writeToFile, this);

  // UNCOMMENT: if UR10e robot used
  // launchWrenchControl();

  // UNCOMMENT: if virtual bouding
  // box is used or UR10e simulator
  launchDoublePoseControl();

  if (m_write_to_file)
    m_thd_wtf.join();
}

/**
 * Destructor
 */
FrankaController::~FrankaController()
{
  std::cout << "Destructor called: " << std::endl;
  m_robot.reset();
  m_model.reset();

  m_moveit_robot_state.reset();
}

/**
 * Sets default collision
 * and impedance behaviour
 */
void FrankaController::setDefaultBehavior()
{
  m_robot->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

  m_robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  m_robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

/**
 * Sets collision behaviour
 */
void FrankaController::setCollisionBehaviour()
{
  m_robot->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
}

/**
 * The following control only computes the
 * joint torques and logs them for the user
 * to check. The torques are not applied to the
 * robot.
 */
void FrankaController::launchSimulatorControl()
{
  m_surface_pose.setIdentity();
  m_current_pose.setIdentity();

  while (run_status)
  {

    std::array<double, 7> dq_array = {0};
    std::array<double, 7> coriolis_array = {0};

    // Get Jacobian
    Eigen::MatrixXd jacobian_;
    Eigen::Vector3d reference_point_position(0.0, -2.0, 0.0);
    auto joint_model_group = m_moveit_robot_model->getJointModelGroup(FRANKA_MODEL_GROUP);
    m_moveit_robot_state->getJacobian(joint_model_group,
                                      m_moveit_robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                      reference_point_position,
                                      jacobian_);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(dq_array.data());

    std::array<double, 7> tau_d_array = computeDPTorque(jacobian, coriolis, dq, true);
    std_msgs::Float64MultiArray array_msg;
    // set up dimensions
    array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    array_msg.layout.dim[0].size = tau_d_array.size();
    array_msg.layout.dim[0].stride = 1;
    array_msg.layout.dim[0].label = "torque forces"; // or whatever name you typically use to index vec1

    // copy in the data
    array_msg.data.clear();
    for (int i = 0; i < tau_d_array.size(); i++)
    {
      array_msg.data.push_back(tau_d_array[i]);
    }
    m_torque_pub.publish(array_msg);
    //ROS_INFO_STREAM("Jacobian: " << jacobian);
    //ROS_INFO("Franka simulator controller: ");
    //ROS_INFO("Torques being applied: ");
    //for (int i = 0; i < tau_d_array.size(); i++)
    //  std::cout << tau_d_array[i] << std::endl;
    ros::spinOnce();
  }
}

/**
 * Define a control behaviour that apply a resistence force
 * between the current and initial pose. This is useful the
 * arm is moving outside the boudary box and trying to drift
 * it back.
 */
void FrankaController::launchDoublePoseControl()
{
  try
  {
    m_surface_pose.setIdentity();
    m_current_pose.setIdentity();

    // Define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        control_callback = [&, this](const franka::RobotState &robot_state,
                                     franka::Duration /*duration*/) -> franka::Torques {
      // Publish robot endeffecor pose
      robotPublisher(robot_state);

      // Get state variables
      std::array<double, 7> coriolis_array = m_model->coriolis(robot_state);
      std::array<double, 42> jacobian_array = m_model->zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // Convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      // Compute torques forces
      std::array<double, 7> tau_d_array = computeDPTorque(jacobian, coriolis, dq, true);

      // Saving robot torques to a class variable, it will
      // be accessed when writing to a file in another thread
      if (m_write_to_file)
      {
        m_mtx_wtf.lock();
        m_robot_state = robot_state;
        m_tau_d_array = tau_d_array;
        m_mtx_wtf.unlock();
      }

      ROS_INFO("Collision was detected due to virtual bouding box or UR10e simulator...");
      ROS_INFO("Torques being applied: ");
      for (int i = 0; i < tau_d_array.size(); i++)
        std::cout << tau_d_array[i] << std::endl;

      // Quit the program
      if (!run_status)
      {
        franka::Torques f_torques(tau_d_array);
        return franka::MotionFinished(f_torques);
      }

      // Default torques return
      return tau_d_array;
    };

    // Start real-time control loop
    m_robot->control(control_callback);
  }
  catch (const franka::Exception &ex)
  {
    // Handle exception
    std::cout << "Control exception: " << ex.what() << std::endl;
  }
}

/**
 * Define a control behaviour that apply a resistance force
 * that is equal to the one felt by the UR10e robotic arm.
 */
void FrankaController::launchWrenchControl()
{
  try
  {
    // Define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        control_callback = [&, this](const franka::RobotState &robot_state,
                                     franka::Duration /*duration*/) -> franka::Torques {
      // Publish robot endeffecor pose
      robotPublisher(robot_state);

      // Get state variables
      std::array<double, 7> coriolis_array = m_model->coriolis(robot_state);
      std::array<double, 42> jacobian_array = m_model->zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // Convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      // Compute control
      Eigen::VectorXd tau_d(7);
      Eigen::VectorXd tau_task(7);

      // Limit velocities
      Eigen::VectorXd velocity_compensation = limit_velocities(dq);

      // Force compensation
      tau_task << jacobian.transpose() * (m_wrench - jacobian * velocity_compensation);
      tau_d << tau_task + coriolis;

      // Send control command
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Apply filtering to torques to avoid
      // excessive torque jumps
      tau_d_array = filter_torques(tau_d_array);

      // Saving robot torques to a class variable, it will
      // be accessed when writing to a file in another thread
      if (m_write_to_file)
      {
        m_mtx_wtf.lock();
        m_robot_state = robot_state;
        m_tau_d_array = tau_d_array;
        m_mtx_wtf.unlock();
      }

      // Quit the program
      if (!run_status)
      {
        franka::Torques torques(tau_d_array);
        return franka::MotionFinished(torques);
      }

      // Default torques return
      return tau_d_array;
    };

    // Start real-time control loop
    m_robot->control(control_callback);
  }
  catch (const franka::Exception &ex)
  {
    // Handle exception
    std::cout << "Control exception: " << ex.what() << std::endl;
  }
}

std::array<double, 7> FrankaController::computeDPTorque(Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian,
                                                        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis,
                                                        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq,
                                                        bool is_simulation)
{
  Eigen::Isometry3d surface_pose(m_surface_pose);
  Eigen::Isometry3d current_pose(m_current_pose);

  // Equilibrium point is the initial position
  Eigen::Vector3d position_s(surface_pose.translation());
  Eigen::Quaterniond orientation_s(surface_pose.linear());

  // Extract current pose information
  Eigen::Vector3d position_c(current_pose.translation());
  Eigen::Quaterniond orientation_c(current_pose.linear());

  Eigen::VectorXd tau_d(7);
  Eigen::VectorXd tau_task(7);

  // Compute translational difference
  // betwen initial pose and current one
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position_c - position_s;

  // Compute rotational difference
  // between initial rotation and current one
  if (orientation_s.coeffs().dot(orientation_c.coeffs()) < 0.0)
  {
    orientation_c.coeffs() << -orientation_c.coeffs();
  }

  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation_c.inverse() * orientation_s);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

  // Transform to base frame
  error.tail(3) << -current_pose.linear() * error.tail(3);

  error *= 10;

  // Cap the values of the error vector
  for (int i = 0; i < error.size(); i++)
  {
    if (std::abs(error[i]) > 0.000001)
    {
      // error[i] += 1 * getSign(error[i]);
      error[i] *= std::abs(error[i]);
    }
  }

  // Compute torques
  if (is_simulation)
  {
    tau_task << jacobian.transpose() * error;
    tau_d << tau_task;
  }
  else
  {
    // Limit velocities
    Eigen::VectorXd velocity_compensation = limit_velocities(dq);

    tau_task << jacobian.transpose() * (-m_stiffness * error - m_damping * (jacobian * velocity_compensation));
    tau_d << tau_task + coriolis;
  }

  // Send control command
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  // Filter tourques in order not
  // to exceed joint max values
  tau_d_array = filter_torques(tau_d_array);

  return tau_d_array;
}

/*
 * Save the wrench forces coming from the 
 * "franka/bounding_limit" topic, to shared variables.
 */
void FrankaController::setWrenchStamped(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  tf::wrenchMsgToEigen(msg->wrench, m_wrench);
}

/*
 * Save the surface and current pose, coming from the 
 * "franka/bounding_limit" topic, to shared variables.
 */
void FrankaController::setPoses(const force_feedback_controller::DoublePose::ConstPtr &msg)
{
  tf::poseMsgToEigen(msg->surface_pose, m_surface_pose);
  tf::poseMsgToEigen(msg->current_pose, m_current_pose);
}

void FrankaController::handleUnexpectedState(const force_feedback_controller::UnexpectedState::ConstPtr &msg)
{
  switch (msg->unexpectedState)
  {
  case ERROR_V_MAX_EXCEEDED:
    std::cout << "exiting due to v_max exceeded in UR10e" << std::endl;
    break;
  case ERROR_SELF_COLLISION:
    std::cout << "exiting due to self collision of UR10e" << std::endl;
    break;
  default:
    std::cout << "strange error detected :-(" << std::endl;
    break;
  }
  raise(SIGINT);
}

/**
 * Publish from file
 */
void FrankaController::filePublisher()
{
  std::string path = ros::package::getPath("force_feedback_controller");
  std::ifstream f_data((path + "/data/franka_data.dump"), std::ifstream::binary);

  // Let's go
  std::string line;
  int count = 0;
  while (ros::ok() && std::getline(f_data, line) && run_status)
  {
    // virtual rate limit for dump
    if (count++ % 100 != 0)
      continue;

    // Extract matrix from the line
    std::array<double, 16> matrix = extract_matrix(line);

    Eigen::Isometry3d goal_pose(Eigen::Matrix4d::Map(matrix.data()));

    // Transform pose into message and publish it
    tf::poseEigenToMsg(goal_pose, m_pose_msg);
    m_ee_pose_pub.publish(m_pose_msg);

    // Spinning
    ros::spinOnce();

    m_loop_rate.sleep();
  }
}

void FrankaController::handleSimulatedEEPose(const geometry_msgs::Pose::ConstPtr &msg)
{
  Eigen::Isometry3d interactive_marker_pose;
  tf::poseMsgToEigen(*msg, interactive_marker_pose);
  std::array<double, 16> offset_panda = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};
  Eigen::Isometry3d panda_goal_pose; //  must not be combined with assignment due to type conversions
  panda_goal_pose = interactive_marker_pose * Eigen::Matrix4d::Map(offset_panda.data()).transpose();
  std::cout << panda_goal_pose.translation() << std::endl;
  trajectory_msgs::JointTrajectory ik_trajectory = performIk(panda_goal_pose, m_moveit_robot_model, m_moveit_robot_state, FRANKA_MODEL_GROUP);
  m_ik_pub.publish(ik_trajectory);
}

/**
 * Publish from robot
 */
int FrankaController::robotPublisher(const franka::RobotState &robot_state)
{
  try
  {
    // Publish EE Pose message
    Eigen::Isometry3d goal_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    tf::poseEigenToMsg(goal_pose, m_pose_msg);
    m_ee_pose_pub.publish(m_pose_msg);

    // Pump callbacks
    ros::spinOnce();
  }
  catch (franka::Exception const &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}

/*
 * Read matrix from line.
 */
std::array<double, 16> FrankaController::extract_matrix(std::string &line)
{
  int numCount = 0;
  int offStart = -1;
  std::array<double, 16> matrix = {0};
  for (unsigned int i = 0; i < line.size(); i++)
  {
    if (line[i] == '[')
    {
      offStart = i + 1;
    }
    else if (line[i] == ',')
    {
      std::string numString = line.substr(offStart, (i - offStart));
      matrix[numCount] = std::stod(numString);
      offStart = i + 1;
      numCount++;
    }
    else if (line[i] == ']')
    {
      std::string numString = line.substr(offStart, (i - offStart));
      matrix[numCount] = std::stod(numString);
      break;
    }
  }
  return matrix;
}

/*
 * Write info to file.
 */
void FrankaController::writeToFile()
{
  std::string f_name = "franka_data_torques.dump";
  std::ofstream file(f_name);

  while (run_status)
  {
    m_mtx_wtf.lock();
    std::array<double, 7> tau_d_array = m_tau_d_array;
    m_mtx_wtf.unlock();

    for (unsigned int i = 0; i < tau_d_array.size(); i++)
    {
      file << tau_d_array[i] << ",";
    }
    file << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  file.close();
}

/*
 * Filter the bounding forces.
 */
std::array<double, 7> FrankaController::filter_torques(std::array<double, 7> curr_torques)
{
  for (unsigned int i = 0; i < curr_torques.size(); i++)
  {
    // applying low pass filter
    m_prev_torques[i] = (1 - A_LOWPASS) * m_prev_torques[i] + A_LOWPASS * curr_torques[i];

    // limit the torques
    if (m_prev_torques[i] < -m_limit_torques[i])
    {
      m_prev_torques[i] = -m_limit_torques[i];
    }
    else if (m_prev_torques[i] > m_limit_torques[i])
    {
      m_prev_torques[i] = m_limit_torques[i];
    }
  }
  return m_prev_torques;
}

/*
 * Limit the velocity instead of shutting down the program.
 */
Eigen::VectorXd FrankaController::limit_velocities(Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq)
{
  Eigen::VectorXd future_velocity(7);
  Eigen::VectorXd ddq(7);

  if (m_first_loop)
  {
    past_dq = dq;
    m_first_loop = false;
  }

  ddq = (dq - past_dq) / TIME_STAMP;

  future_velocity = (dq + ddq * TIME_STAMP) * 1.2;

  for (int i = 0; i < 7; i++)
  {
    if (i < 4)
    {
      if (future_velocity[i] < 2.0 && future_velocity[i] > -2.0)
      {
        future_velocity[i] = 0;
      }
      else
      {
        run_status = false;
      }
    }
    else
    {
      if (future_velocity[i] < 2.6 && future_velocity[i] > -2.6)
      {
        future_velocity[i] = 0;
      }
      else
      {
        run_status = false;
      }
    }
  }

  past_dq = dq;
  return future_velocity;
}

/**
 * Main function
 */
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "franka_controller", ros::init_options::NoSigintHandler);

  // Ctrl C handler
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_handler = &quitHandler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, &old_action);

  // Sping controllers
  if (argc != 1 && argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  else if (argc == 1)
  {
    std::cout << "FrankaController: starting with control by interactive marker" << std::endl;
    FrankaController franka_controller(true);
  }
  else
  {
    // Get address
    std::string address = argv[1];

    // Create robot instance
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);

    // Create model instance
    franka::Model model = robot.loadModel();

    // Start controller
    FrankaController franka_controller(address, robot, model);
  }

  return 0;
}
