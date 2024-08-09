// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <limits>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // init SEDS 
  Prior.setZero();
  Mu.setZero();
  //Sigma_flatten.setZero();
  //att.setZero();

  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
      
}

void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

  // Start to record or to reproduce
  if(recording == false && reproduction == false){
    std::cout << "Please enter the demonstration number or type 99 to run reproduced trajectory:" << std::endl;
    std::cin >> demo_num;
    if (demo_num == 99)
    {
      std::cout << "Reproduction running" << std::endl;
      reproduction = true;

    }else{
      std::cout << "Your demos dumber is " << demo_num << std::endl;
      std::string filename = "/home/panda/YanQu/MA/Demo_data_2/follower_"+std::to_string(demo_num)+".txt";
      follower_file.open(filename);
      if (!follower_file){
        ROS_ERROR("Failed to write the data");
        //return false;
      }
      follower_file << "Xl_x\tXl_y\tXl_z\tVl_x\tVl_y\tVl_z\n";
      recording = true;
    }
  }

  // record the position and velocity
  if (follower_file.is_open() && recording == true){
    std::cout << "Recording position and velocities" <<std::endl;
    follower_file << position[0] << "\t" << position[1] << "\t" << position[2] << "\t" << velocity[0] << "\t" << velocity[1]<< "\t" << velocity[2]<<"\n";
  }
  
  // reproduce the trajectory
  if (reproduction == true){
    // Read Prior, Mu, Sigma and attractor 
    std::vector<double> prior = readCsv("/home/panda/YanQu/MA/Learn_data/priors.csv", nbStates, 1);
    std::vector<double> mu = readCsv("/home/panda/YanQu/MA/Learn_data/mu.csv", 6, nbStates);
    std::vector<double> sigma = readCsv("/home/panda/YanQu/MA/Learn_data/sigma.csv", 36, nbStates);
    //const Eigen::Vector3d att = {0.3739, -0.4064, 0.3662};
    const Eigen::Vector3d att = {0.5461, -0.0545, 0.0596};

    Eigen::Map<Eigen::Matrix<double, 4, 1>> Prior(prior.data());
    Eigen::Map<Eigen::Matrix<double, 4, 6>> Mu(mu.data());
    Eigen::Map<Eigen::Matrix<double, 4, 36>> Sigma_temp(sigma.data());
    Eigen::Matrix<double,36,4> Sigma_flatten = Sigma_temp.transpose();

    std::vector<Eigen::MatrixXd> Sigma(4,Eigen::MatrixXd(6,6));

    for (int i = 0; i < nbStates; ++i) {
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
                Sigma[i](row, col) = Sigma_flatten(row * 6 + col, i);
            }
        }
    }
    // std::cout<<"Prior: "<< Prior<<std::endl;
    // std::cout<<"Mu:    "<< Mu<<std::endl;
    // std::cout<<"Sigma: "<< Sigma[0]<<std::endl;
    Eigen::Matrix<double, 3, 1> diff;
    Eigen::VectorXd Pxi(nbStates);
    Eigen::VectorXd beta(nbStates);
    Eigen::Vector3d velocity_d;
    //velocity_d.setZero();

    for (int i = 0; i < nbStates; ++i) {
      int nbVar = position.size();

      // Gauss PDF
      diff = position - att - Mu.transpose().col(i).head(3);
      double prob = diff.transpose() * Sigma[i].topLeftCorner(3,3).inverse() * diff;
      prob = exp(-0.5 * prob) / sqrt(pow(2 * M_PI, nbVar) * Sigma[i].topLeftCorner(3,3).determinant() + std::numeric_limits<double>::min());
      Pxi[i] = Prior[i]* prob;
    }
    double sumPxi = Pxi.sum() + std::numeric_limits<double>::min();
    beta = Pxi / sumPxi;

    for (int j = 0; j < nbStates; j++)
    {
      Eigen::VectorXd yj_tmp = Mu.transpose().col(j).tail(3) + Sigma[j].bottomLeftCorner(3, 3) *
                                  Sigma[j].topLeftCorner(3, 3).inverse() * (position - att - Mu.transpose().col(j).head(3));
      velocity_d = velocity_d + beta(j) * yj_tmp;
    }
    dt = 0.001;
    position_d_[0]= position[0] + velocity_d[0] * dt;
    position_d_[1]= position[1] + velocity_d[1] * dt;
    position_d_[2]= position[2] + velocity_d[2] * dt;
    
    std::cout<< "velocity_d: "<< velocity_d[0] <<" "<< velocity_d[1] <<" "<< velocity_d[2] << std::endl;
    std::cout<< "position_d: "<< position_d_[0]<<" "<< position_d_[1]<<" "<< position_d_[2]<< std::endl;
    std::cout<< "position:   "<< position[0]   <<" "<< position[1]   <<" "<< position[2]<< std::endl;    
  }
  
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  cartesian_stiffness_.topLeftCorner(3, 3) << 800.0 * Eigen::MatrixXd::Identity(3, 3);
  cartesian_stiffness_.bottomRightCorner(3, 3) << 20.0 * Eigen::MatrixXd::Identity(3, 3);
  cartesian_damping_.topLeftCorner(3, 3) << 2.0 * sqrt(800) * Eigen::Matrix3d::Identity();

 // Cartesian PD control with damping ratio = 1  
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  
  // Kinematic operation to record data
  if(recording == true){
    tau_d.setZero();
  }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceExampleController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

std::vector<double> readCsv(const std::string& filename, int numRows, int numCols) {
    std::vector<double> matrix;
    matrix.reserve(numRows * numCols);
    std::ifstream file(filename);
    double num;

    while (file >> num) {
        matrix.push_back(num);
        // Skip the comma
        file.ignore();
    }

    return matrix;
}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
