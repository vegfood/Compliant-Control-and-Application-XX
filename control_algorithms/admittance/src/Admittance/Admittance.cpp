/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:47 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:47 
 * @Licence: MIT Licence
 */
#include <admittance/Admittance.h>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
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
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link){

  //* Subscribers
  sub_arm_state_           = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_desired_        = nh_.subscribe(topic_wrench_desired, 5,
                                             &Admittance::desired_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_desired_state_        = nh_.subscribe(topic_desired_state, 5,
                                             &Admittance::state_desired_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  //* Publishers
  pub_arm_cmd_              = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  // initializing the class variables
  arm_position_.setZero();
  arm_twist_.setZero();
  wrench_external_.setZero();
  wrench_desired_.setZero();
  integral_force_error.setZero();
  last_force_error.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();



  while (nh_.ok() && !arm_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // Init integrator
  arm_desired_twist_adm_.setZero();
  arm_desired_twist.setZero();
  arm_desired_acceleration.setZero();


  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  force_x_pre = 0;
  force_y_pre = 0;
  force_z_pre = 0;
  wait_for_transformations();
}

//!-                   INITIALIZATION                    -!//

void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
  base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
  world_arm_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void Admittance::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {

//    compute_admittance();
    compute_hybrid_control();
    send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//

void Admittance::compute_admittance() {

  error.topRows(3) = arm_position_ - desired_pose_position_;
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm;

  coupling_wrench_arm=   D_ * (arm_twist_ - arm_desired_twist) + K_*error;
//    arm_desired_acceleration = M_.inverse() * (- coupling_wrench_arm + wrench_external_);
//arm twist in represented in base_Link
  arm_desired_acceleration_adm_ = M_.inverse() * (- coupling_wrench_arm + wrench_external_) + arm_desired_acceleration;


//  double a_acc_norm = (arm_desired_acceleration.segment(0, 3)).norm();

//  if (a_acc_norm > arm_max_acc_) {
//    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
//                             << " norm: " << a_acc_norm);
//    arm_desired_acceleration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
//  }
//  else {
//      ROS_WARN_STREAM_THROTTLE(1, "Admittance generates [normal] arm accelaration!"
//              << " norm: " << a_acc_norm);
//  }
  double a_acc_norm = (arm_desired_acceleration_adm_.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_acceleration_adm_.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }
  else {
      ROS_WARN_STREAM_THROTTLE(1, "Admittance generates [normal] arm accelaration!"
              << " norm: " << a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
//  arm_desired_twist_adm_ += arm_desired_acceleration * duration.toSec();
//  arm_desired_twist_adm_ = arm_desired_acceleration * duration.toSec() + arm_twist_;
  arm_desired_twist_adm_ = arm_desired_acceleration_adm_ * duration.toSec() + arm_twist_;


}

//!-                Hybrid Force/Position Control        -!//

void Admittance::compute_hybrid_control() {

    // TODO:enable to set desired wrench by topic,add real force sensor in ee_link;
    // Integrate for velocity based interface
    ros::Duration duration = loop_rate_.expectedCycleTime();

    // 定义选择矩阵，1表示力控制，0表示位置控制
    Eigen::Matrix<double, 6, 6> selectionMatrix;
    selectionMatrix.setZero();
    for(int i=2; i < 6; i++){
        selectionMatrix(i, i) = 1;
//        selectionMatrix(i, i) = main_force_control_axis(i); // 力主控制轴使用力控制，其他轴使用位置控制
    }
    //将末端坐标系的选择矩阵转换到基坐标系下
    Matrix6d rotation_ft_base;
    get_rotation_matrix(rotation_ft_base, listener_ft_, end_link_, base_link_);
//    ROS_WARN_STREAM_THROTTLE(1, "rotation_ft_base:" << rotation_ft_base);
    Matrix6d rotation_base_ft;
    get_rotation_matrix(rotation_base_ft, listener_ft_, base_link_, end_link_);
//    ROS_WARN_STREAM_THROTTLE(1, "rotation_base_ft:" << rotation_ft_base);
//    ROS_WARN_STREAM_THROTTLE(1, "selectionMatrix:" << selectionMatrix);
    // 计算位置误差和力误差
    error.topRows(3) = arm_position_ - desired_pose_position_;
    if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
    {
        arm_orientation_.coeffs() << -arm_orientation_.coeffs();
    }
    Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
    if(quat_rot_err.coeffs().norm() > 1e-3)
    {
        quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
    }
    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
    error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

    // 力控制计算
    // PID 控制器增益
    double Kp = 1; // 比例增益
    double Ki = 0.0; // 积分增益
    double Kd = 0; // 微分增益
    double integral_limit = 1000.0; // 积分限制

    Vector6d force_error = - wrench_external_ + wrench_desired_; //末端坐标系下
    // 积分项
//    integral_force_error += force_error * duration.toSec();
    integral_force_error += force_error;

    // 积分项计算，考虑积分限制
//    for (int i = 0; i < 6; ++i) {
//        if (integral_force_error(i) > integral_limit) {
//            integral_force_error(i) = integral_limit;
//        } else if (integral_force_error(i) < -integral_limit) {
//            integral_force_error(i) = -integral_limit;
//        }
//    }
    //微分项
//    Vector6d d_force_error = (force_error - last_force_error) / duration.toSec();
    Vector6d d_force_error = (force_error - last_force_error);

    last_force_error = force_error;

    ROS_WARN_STREAM_THROTTLE(1, "wrench external!"
            << " norm: \n" << wrench_external_);
    ROS_WARN_STREAM_THROTTLE(1, "wrench desired!"
            << " norm: \n" << wrench_desired_);
    ROS_WARN_STREAM_THROTTLE(1, "force_error!"
            << " norm: \n" << force_error);
    //力控制输出
    Vector6d force_control_output =  -(Kp * force_error + Ki * integral_force_error + Kd * d_force_error);

    // 位置控制输出
//    Vector6d position_control_output = -(D_ * (arm_twist_ - arm_desired_twist) + K_*error) + rotation_ft_base * wrench_external_; //添加阻抗
    Vector6d position_control_output = -(D_ * (arm_twist_ - arm_desired_twist) + K_*error) ; //纯位置控制，不加阻抗

    //根据力控误差的情况确定力控和位控的维度
    int force_error_limit = 5;
    double torque_error_limit = 0.2;
    double error_limit; // 根据 i 的值选择 force_error_limit 或 torque_error_limit
    for (int i = 3; i < 6; i++) {
        if (i < 3) {
            error_limit = force_error_limit;
        } else {
            error_limit = torque_error_limit;
        }


        if (abs(force_error(i)) < error_limit && main_force_control_axis(i) < 1) {
            selectionMatrix(i, i) = 0;

        } else {
            bool equal = (main_force_control_axis(i) == 0);
            ROS_WARN_STREAM_THROTTLE(1, "abs(force_error(i)) \n" << abs(force_error(i)));
            ROS_WARN_STREAM_THROTTLE(1, "main_force_control_axis \n" << main_force_control_axis(i));
            ROS_WARN_STREAM_THROTTLE(1, "equal \n" << equal);

            selectionMatrix(i, i) = 1;
        }
    }

    ROS_WARN_STREAM_THROTTLE(1, "selectionMatrix \n" << selectionMatrix);
    // 混合控制输出
    Vector6d control_output = rotation_ft_base * selectionMatrix *  force_control_output +
                              rotation_ft_base * (Matrix6d::Identity() - selectionMatrix) * rotation_base_ft * position_control_output;
//    Vector6d control_output = rotation_ft_base * selectionMatrix * rotation_base_ft * force_control_output +
//            rotation_ft_base * (Matrix6d::Identity() - selectionMatrix) * rotation_base_ft * position_control_output;
//    Vector6d control_output = selectionMatrix * force_control_output;

    // 计算加速度
    //arm twist in represented in base_Link
    arm_desired_acceleration = M_.inverse() * control_output;

    double a_acc_norm = (arm_desired_acceleration.segment(0, 3)).norm();
    if (a_acc_norm > arm_max_acc_) {
        ROS_WARN_STREAM_THROTTLE(1, "Hybrid control generates high arm acceleration!"
                << " norm: " << a_acc_norm);
        arm_desired_acceleration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
    }
    else {
        ROS_WARN_STREAM_THROTTLE(1, "Hybrid control generates [normal] arm acceleration!"
                << " norm: " << a_acc_norm);
    }

    arm_desired_twist_adm_ = arm_desired_acceleration * duration.toSec() + arm_twist_;
}


//!-                     CALLBACKS                       -!//

void Admittance::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_position_ <<  msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;

  arm_orientation_.coeffs() <<  msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;

  arm_twist_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}

void Admittance::state_desired_callback(
        const cartesian_state_msgs::PoseTwistConstPtr msg) {
    desired_pose_position_ <<  msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    desired_pose_orientation_.coeffs() <<  msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w;

    arm_desired_twist << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z,
            msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z;
}

void Admittance::state_wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    wrench_ft_frame <<  msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,msg->wrench.torque.x,
    msg->wrench.torque.y,msg->wrench.torque.z;
//      wrench_ft_frame <<  0,0,msg->wrench.force.z,0,0,0;

    float force_thres_lower_limit_ = 50;
    float force_thres_upper_limit_ = 100;

//     Low-Pass Filter for real robot
//     if(fabs(wrench_ft_frame(0)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(0)) > force_thres_upper_limit_){wrench_ft_frame(0) = 0;}
//     else{
//       if(wrench_ft_frame(0) > 0){wrench_ft_frame(0) -= force_thres_lower_limit_;}
//       else{wrench_ft_frame(0) += force_thres_lower_limit_;}
//       wrench_ft_frame(0) = (1 - 0.2)*force_x_pre + 0.2*wrench_ft_frame(0);
//       force_x_pre = wrench_ft_frame(0);
//     }
//     if(fabs(wrench_ft_frame(1)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(1)) > force_thres_upper_limit_){wrench_ft_frame(1) = 0;}
//     else{
//       if(wrench_ft_frame(1) > 0){wrench_ft_frame(1) -= force_thres_lower_limit_;}
//       else{wrench_ft_frame(1) += force_thres_lower_limit_;}
//       wrench_ft_frame(1) = (1 - 0.2)*force_y_pre + 0.2*wrench_ft_frame(1);
//       force_y_pre = wrench_ft_frame(1);
//     }
//     if(fabs(wrench_ft_frame(2)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(2)) > force_thres_upper_limit_){wrench_ft_frame(2) = 0;}
//     else{
//       if(wrench_ft_frame(2) > 0){wrench_ft_frame(2) -= force_thres_lower_limit_;}
//       else{wrench_ft_frame(2) += force_thres_lower_limit_;}
//       wrench_ft_frame(2) = (1 - 0.2)*force_z_pre + 0.2*wrench_ft_frame(2);
//       force_z_pre = wrench_ft_frame(2);
//     }
//    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
//    get_rotation_matrix(rotation_ft_base, listener_ft_, end_link_, base_link_);

//this force is represented is base_link
//    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;

//this force is represented in eef link
    wrench_external_ << wrench_ft_frame;
  }
}

void Admittance::desired_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg) {
    if (ft_arm_ready_) {
        //this force is wrt end_link
    main_force_control_axis << 0,0,1,0,0,0;
    wrench_desired_ <<  0,0,msg->wrench.force.z,0,0,0;
    Matrix6d rotation_ft_base;
    get_rotation_matrix(rotation_ft_base, listener_ft_, end_link_, base_link_);
    //convert this force into base_link
//    wrench_desired_ <<  rotation_ft_base * wrench_desired_;

    }
}

//!-               COMMANDING THE ROBOT                  -!//

void Admittance::send_commands_to_robot() {
  // double norm_vel_des = (arm_desired_twist_adm_.segment(0, 3)).norm();

  // if (norm_vel_des > arm_max_vel_) {
  //   ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

  //   arm_desired_twist_adm_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  // }
  geometry_msgs::Twist arm_twist_cmd;
//  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0)*0.3;
//  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1)*0.3;
//  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2)*0.3;
//  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3)*0.3;
//  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4)*0.3;
//  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5)*0.3;

   arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
   arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
   arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
   arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
   arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
   arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);
  pub_arm_cmd_.publish(arm_twist_cmd);
}

//!-                    UTILIZATION                      -!//

bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}

