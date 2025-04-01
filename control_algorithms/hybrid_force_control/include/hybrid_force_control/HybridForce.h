/*
 * @Author: MingshanHe
 * @Date: 2021-12-05 04:08:00
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:08:21
 * @Licence: MIT Licence
 */

#pragma once

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

class HybridForce {
protected:
    // ROS VARIABLES:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // CONTROLLER PARAMETERS:
    //先默认构造为3x3矩阵
    MatrixXd K_i_v_, K_i_lambda_, K_p_lambda_;
    //任务坐标系相对于基坐标系的刚体变换矩阵
    Isometry3d  T_task_base;
    //构造任务坐标系到基坐标系的6x6变换矩阵
    Matrix6d trans_task_base;
    //力的维度和速度的维度对应的选择矩阵
    Eigen::MatrixXd S_v_, S_f_;
    Eigen::MatrixXd S_v_inv_, S_f_inv_;
    //力选择矩阵的伪逆对应的权重，W_f=K^-1;速度选择矩阵的伪逆对应的权重，W_v=M;
    Matrix6d W_f_;
    Matrix6d W_v_;
    //环境刚度K_env_
    Matrix6d K_env_;
    //柔顺矩阵C_prime
    Matrix6d C_prime_;


    // Subscribers:
    ros::Subscriber sub_arm_state_;
    ros::Subscriber sub_wrench_state_;
    ros::Subscriber sub_desired_state_;
    ros::Subscriber sub_wrench_desired_;

    // Publishers:
    ros::Publisher pub_arm_cmd_;

    // Variables:
    //当前机械臂末端的位姿和速度
    Vector3d arm_position_;
    Quaterniond arm_orientation_;
    Vector3d arm_last_position_;
    Quaterniond arm_last_orientation_;
    double last_arm_twist_time_;
    //v_e
    Vector6d arm_twist_;

    //末端测量的外力，与交互力h_e的方向相反,
    Vector6d wrench_external_;
    //控制器输入： 期望的外力, 默认表达在任务坐标系
    Vector6d wrench_desired_threshold_;
    //todo:添加期望力的导数，实现非恒力的跟踪

    // 控制器输入：末端坐标系期望速度，v_d of desired frame， 默认表达在任务坐标系
    Vector6d arm_desired_velocity_twist;
    // 期望保持的末端姿态
    Vector7d arm_desired_pose_;
    Quaterniond  arm_desired_orientation_;

    // 期望速度和实际速度的积分累积误差
    VectorXd v_error_integral;

    // 期望力和实际力的积分累积误差
    VectorXd force_error_integral;

    // TF:
    // Transform from base_link to world
    Matrix6d rotation_base_;
    // Listeners
    tf::TransformListener tf_listener_;

    // Guards
    bool ft_arm_ready_;
    bool arm_twist_flag_;

    double arm_max_vel_;
    double arm_max_acc_;

public:
    HybridForce(ros::NodeHandle &n, double frequency,
                const std::string &topic_arm_state,
                const std::string &topic_arm_command,
                const std::string &topic_wrench_state,
                const std::string &topic_wrench_desired,
                const std::string &topic_desired_state,
                std::vector<double> K_i_v,
//                std::vector<double> K_p_v,
                std::vector<double> K_i_lambda,
                std::vector<double> K_p_lambda,
                std::vector<double> S_v,
                std::vector<double> S_f,
                std::vector<double> W_v,
                std::vector<double> W_f,
                std::vector<double> K_env,
                std::vector<double> desired_pose,
                std::string base_link,
                const std::string &end_link,
                std::string interface_type,
                double arm_max_vel,
                double arm_max_acc
    );

    ~HybridForce() {}

    void run();

private:
    //初始化 选择矩阵
    void init_selection_matrix(MatrixXd& S, std::vector<double>& vec);
    //初始化 P，I参数矩阵
    void init_control_matrix(const MatrixXd& S, std::vector<double>& vec, MatrixXd& K);
    // Control
    Vector6d compute_hybrid_force_velocity_interface();

    // Callbacks
    void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr &msg);

    void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr &msg);

    void desired_wrench_callback(const geometry_msgs::WrenchStampedConstPtr &msg);

    void desired_state_callback(const cartesian_state_msgs::PoseTwistConstPtr &msg);

    //
    void send_commands_to_robot(const Vector6d &cmd);

    //
    void wait_for_transformations();

    static bool get_rotation_matrix(Matrix6d &rotation_matrix,
                                    tf::TransformListener &listener,
                                    const std::string &from_frame, const std::string &to_frame);

private:
    std::string base_link_;
    std::string end_link_;
    std::string ft_frame_;

    // Controller interface type
    std::string interface_type_;

};

