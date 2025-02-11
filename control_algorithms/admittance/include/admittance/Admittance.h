/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:00 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:08:21
 * @Licence: MIT Licence
 */

#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
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
  Matrix6d M_, D_, K_;

  // Subscribers:
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_wrench_state_;
  ros::Subscriber sub_desired_state_;
  ros::Subscriber sub_wrench_desired_;

  // Publishers:
  ros::Publisher pub_arm_cmd_;

  // Variables:
  //当前机械臂末端的位姿和速度
  Vector3d      arm_position_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;

  //末端测量的外力，与交互力h_e的方向相反
  Vector6d      wrench_external_;
  //期望的外力, 默认方向与h_e方向相同
  Vector6d      wrench_desired_;

  // 导纳控制器输出：末端柔顺坐标系的速度，v_c of compliant frame
  Vector6d      arm_desired_twist_adm_;
  // 导纳控制器输出：末端柔顺坐标系的加速度，a_c of compliant frame
  Vector6d      arm_desired_acceleration_adm_;
  // 导纳控制器输出：末端柔顺坐标系的位姿，x_c of compliant frame
  Vector3d      desired_pose_position_adm_;
  Quaterniond   desired_pose_orientation_adm_;

  // 导纳控制器输入：末端坐标系期望加速度，a_d of desired frame
  Vector6d      arm_desired_acceleration;
  // 导纳控制器输入：末端坐标系期望速度，v_d of desired frame
  Vector6d      arm_desired_twist;

  // 导纳控制器输入：末端坐标系期望位姿，x_d of desired frame
  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;

  //柔顺坐标系的位姿x_c与期望位姿x_d的差值，error = x_d - x_c
  Vector6d      error;
  Vector6d      integral_force_error;
  Vector6d      last_force_error;

  Vector6d  main_force_control_axis;


  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  double arm_max_vel_;
  double arm_max_acc_;

  double force_x_pre, force_y_pre, force_z_pre;

public:
  Admittance(ros::NodeHandle &n, double frequency,
                      std::string topic_arm_state,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::string topic_wrench_desired,
                      std::string topic_desired_state,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc
                       );
  ~Admittance(){}
  void run();
private:
  // Control
  void compute_admittance();
  void compute_hybrid_control();
  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void desired_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void state_desired_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

  // 
  void send_commands_to_robot();

  // 
  void wait_for_transformations();
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
private:
  std::string   base_link_;
  std::string   end_link_;
  std::string   control_frame_;
};

#endif // ADMITTANCE_H