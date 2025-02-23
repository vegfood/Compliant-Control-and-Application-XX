#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
#define PI 3.1415926

class Admittance
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  // ADMITTANCE PARAMETERS:
  Matrix6d M_, D_, K_, Dcar_;

  // Subscribers:
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_wrench_state_;
  // Publishers:
  ros::Publisher  pub_arm_cmd_;
  ros::Publisher  pub_car_cmd_;

  // Variables:
  Vector3d      arm_position_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;
  Vector6d      wrench_external_;
  Vector6d      arm_desired_velocity_twist_adm_;
  Vector6d      arm_desired_acceleration;
  Vector6d      car_desired_twist_adm_;
  Vector6d      car_desired_accelaration;

  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;

  Vector6d      error;

  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener tf_listener_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  double arm_max_vel_;
  double arm_max_acc_;

public:
  Admittance(ros::NodeHandle &n, double frequency,
                      std::string topic_arm_state,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::string topic_car_command,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      std::vector<double> Dcar,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc
                       );
  ~Admittance(){}
  void run();
private:

  void compute_admittance_position_interface();

  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

  void state_wrench_callback(const geometry_msgs::Wrench msg);

  void send_commands_to_robot();

  void send_commands_to_car();

  void wait_for_transformations();

  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
private:
  std::string   base_link_;
  std::string   end_link_;
};

#endif // ADMITTANCE_H