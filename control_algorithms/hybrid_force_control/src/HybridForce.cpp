#include "hybrid_force_control/HybridForce.h"

#include <utility>

HybridForce::HybridForce(ros::NodeHandle &n, double frequency,
                         const std::string &topic_arm_state,
                         const std::string &topic_arm_command,
                         const std::string &topic_wrench_state,
                         const std::string &topic_wrench_desired,
                         const std::string &topic_desired_state,
                         std::vector<double> K_i_v,
//                         std::vector<double> K_p_v,
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
                         double arm_max_acc) :
        nh_(n), loop_rate_(frequency),
//        K_i_v_(K_i_v.data()), K_i_lambda_(K_i_lambda.data()), //K_p_v_(K_p_v.data()),
//        K_p_lambda_(K_p_lambda.data()),
        W_f_(W_f.data()), W_v_(W_v.data()), K_env_(K_env.data()),
        arm_desired_pose_(desired_pose.data()),
        arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
        base_link_(std::move(base_link)), end_link_(end_link), ft_frame_(end_link),
        interface_type_(std::move(interface_type)) {
    T_task_base = Isometry3d::Identity();
    trans_task_base = Matrix6d::Zero();
    // 将 R 放在对角线上
    trans_task_base.topLeftCorner(3, 3) = T_task_base.linear();
    trans_task_base.bottomRightCorner(3, 3) = T_task_base.linear();

    //初始化选择矩阵
    init_selection_matrix(S_v_, S_v);
    init_selection_matrix(S_f_, S_f);
    S_v_inv_ = (S_v_.transpose() * W_v_ * S_v_).inverse() * S_v_.transpose() * W_v_;
    S_f_inv_ = (S_f_.transpose() * W_f_ * S_f_).inverse() * S_f_.transpose() * W_f_;

    //初始化P、I矩阵
    init_control_matrix(S_v_, K_i_v, K_i_v_);
    init_control_matrix(S_f_, K_i_lambda, K_i_lambda_);
    init_control_matrix(S_f_, K_p_lambda, K_p_lambda_);

    //初始化柔顺矩阵
    C_prime_ = K_env_.inverse() - S_v_ * S_v_inv_ * K_env_.inverse();

    //* Subscribers
    sub_arm_state_ = nh_.subscribe(topic_arm_state, 5,
                                   &HybridForce::state_arm_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_state_ = nh_.subscribe(topic_wrench_state, 5,
                                      &HybridForce::state_wrench_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_desired_ = nh_.subscribe(topic_wrench_desired, 5,
                                        &HybridForce::desired_wrench_callback, this,
                                        ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_state_ = nh_.subscribe(topic_desired_state, 5,
                                       &HybridForce::desired_state_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());
    //* Publishers
    if (interface_type_ == "velocity") {
        pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

    } else {
        pub_arm_cmd_ = nh_.advertise<geometry_msgs::Pose>(topic_arm_command, 5);

    }

    // initializing the class variables
    arm_position_.setZero();
    arm_twist_.setZero();
    wrench_external_.setZero();
    wrench_desired_threshold_.setZero();
    arm_desired_velocity_twist.setZero();
    arm_desired_orientation_.coeffs() << arm_desired_pose_.bottomRows(4)/arm_desired_pose_.bottomRows(4).norm();


    while (nh_.ok() && !arm_position_(0)) {
        ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
        ros::spinOnce();
        loop_rate_.sleep();
    }

    // Init integrator
    v_error_integral.resize(K_i_v_.cols());
    v_error_integral.setZero();
    force_error_integral.resize(K_i_lambda_.cols());
    force_error_integral.setZero();

    //init ft sensor frame flag
    ft_arm_ready_ = false;

    wait_for_transformations();
}

//
void HybridForce::init_selection_matrix(MatrixXd &S, std::vector<double> &vec) {
    int n = static_cast<int>(vec.size() / 6);
    S = Eigen::Matrix<double, 6, Eigen::Dynamic>(6, n);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i * n + j < vec.size()) {
                S(i, j) = vec[i * n + j];
//            } else {
//                S(i, j) = 0; // 用零填充剩余部分
//            }
            }
        }

    }
}

void HybridForce::init_control_matrix(const MatrixXd& S, std::vector<double>& vec, MatrixXd& K){
    auto dimension = S.cols();
    K = MatrixXd::Zero(dimension, dimension);
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            if (i * dimension + j < vec.size()) {
                K(i, j) = vec[i * dimension + j];
            }
        }
    }
}

//!-                   INITIALIZATION                    -!//

void HybridForce::wait_for_transformations() {
    tf::TransformListener listener;
    Matrix6d rot_matrix;
    while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) { sleep(1); }
    ft_arm_ready_ = true;
    ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void HybridForce::run() {

    ROS_INFO("Running the hybrid force control loop .................");

    while (nh_.ok()) {
        if (interface_type_ == "velocity") {
            Vector6d cmd = compute_hybrid_force_velocity_interface();
            send_commands_to_robot(cmd);

        } else {
//        Vector7d cmd = compute_admittance_position_interface();
//            Vector7d cmd = compute_admittance_simplified_position_interface();
//            send_commands_to_robot(cmd);
        }

        ros::spinOnce();
        loop_rate_.sleep();
    }
}

//!-                基于速度接口的混合力控实现                  -!//
Vector6d HybridForce::compute_hybrid_force_velocity_interface() {
    //基于内环为速度的力位混合控制器：V_r = S_v * V_v + C' * S_f * f_lambda
    //为了简化处理，选择末端坐标系作为控制坐标系,即control frame 等于 end_frame
    //todo:实现基于速度内环的力位混合控制
    // V_v = V_d + K_iv * Integral(V_d - V_c)
    // 求解基坐标系在任务坐标系下的旋转矩阵
    // 统一变换到任务坐标系, 期望的速度和力都是表示在任务坐标系
    //获得末端的实时速度：由于位置接口无法返回准确的速度
    VectorXd V_c = S_v_inv_ * trans_task_base.inverse() * arm_twist_;
    ROS_WARN_STREAM_THROTTLE(1, "current cartesian velocity:" << V_c);
    //计算姿态误差
    Vector6d pose_error;
    pose_error.setZero();
    if(arm_orientation_.coeffs().dot(arm_desired_orientation_.coeffs()) < 0.0)
    {
        arm_orientation_.coeffs() << -arm_orientation_.coeffs();
    }
    Eigen::Quaterniond quat_rot_err (arm_desired_orientation_.inverse() * arm_orientation_);
    if(quat_rot_err.coeffs().norm() > 1e-3)
    {
        quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
    }
    //表示在基坐标系下的相对姿态误差
    Matrix3d R_desired_base = arm_desired_orientation_.toRotationMatrix();
    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
    pose_error.bottomRows(3) << - R_desired_base * err_arm_des_orient.axis() * err_arm_des_orient.angle();
    //期望速度+末端姿态跟踪
//    VectorXd V_d = S_v_inv_ * (arm_desired_velocity_twist + pose_error);
    //期望速度
    VectorXd V_d = S_v_inv_ * arm_desired_velocity_twist;

    ROS_WARN_STREAM_THROTTLE(1, "desired cartesian velocity:" << V_d);
    //控制周期
    ros::Duration duration = loop_rate_.expectedCycleTime();
    v_error_integral.resize(V_d.size());
    ROS_WARN_STREAM_THROTTLE(1, "integral cartesian velocity error:" << v_error_integral);
    v_error_integral += (V_d - V_c) * duration.toSec();

    VectorXd V_v = V_d + K_i_v_ * v_error_integral;
    //f_lambda = dot_lamdda_d(恒力为零) + K_p_lambda * [lambda_d - lambda_c] + K_i_lamda * Integral(lambda_d - lambda_c)
    // 获取末端力矩传感器到基座的变换，再变换到任务坐标系
    Matrix6d rot_ft_base;
    get_rotation_matrix(rot_ft_base, tf_listener_, ft_frame_, base_link_);
    Matrix6d rot_ft_task = trans_task_base.inverse() * rot_ft_base;
    VectorXd lambda_c = S_f_inv_ * rot_ft_task * (- wrench_external_);
    ROS_WARN_STREAM_THROTTLE(1, "current force in task frame:" << lambda_c);
    //期望的lambda_d 表示为机械臂末端期望对环境施加的力
    VectorXd lambda_d = S_f_inv_ * wrench_desired_threshold_;
    ROS_WARN_STREAM_THROTTLE(1, "desired force in task frame:" << lambda_d);
    force_error_integral.resize(lambda_d.size());
    force_error_integral += (lambda_d - lambda_c) * duration.toSec();
    ROS_WARN_STREAM_THROTTLE(1, "integral force error:" << force_error_integral);
    VectorXd f_lambda = K_p_lambda_ * (lambda_d - lambda_c) + K_i_lambda_ * (lambda_d - lambda_c);
    ROS_WARN_STREAM_THROTTLE(1, "force control output:" << f_lambda);

    VectorXd V_r = S_v_ * V_v + C_prime_ * S_f_ * f_lambda;
    // 再变换为基坐标系下的速度
    Vector6d V_cmd = trans_task_base * V_r;
    ROS_WARN_STREAM_THROTTLE(1, "HybridForce generates cartesian velocity cmd:" << V_cmd);
    // 方法一：假设柔顺环境，机械臂的末端速度等于控制指令，即v_real(arm_twist) = v_r(v_cmd)
//    arm_twist_ = V_cmd;
    // 方法二：根据位置微分

    return V_cmd;
}


//!-                     CALLBACKS                       -!//

void HybridForce::state_arm_callback(
        const cartesian_state_msgs::PoseTwistConstPtr &msg) {
    arm_position_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    arm_orientation_.coeffs() << msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w;
//
//    arm_twist_ << msg->twist.linear.x,
//            msg->twist.linear.y,
//            msg->twist.linear.z,
//            msg->twist.angular.x,
//            msg->twist.angular.y,
//            msg->twist.angular.z;
    //根据位置微分计算速度
    if (arm_twist_flag_){
        auto delta_t = msg->header.stamp.toSec() - last_arm_twist_time_;
        auto delta_pos = arm_position_ - arm_last_position_;
        auto delta_q = arm_orientation_ * arm_last_orientation_.inverse();
        AngleAxisd delta_angle(delta_q);
        auto trans_v = delta_pos / delta_t;
        auto rot_v = delta_angle.angle() / delta_t * delta_angle.axis();
        arm_twist_ << trans_v[0], trans_v[1], trans_v[2], rot_v[0], rot_v[1], rot_v[2];
    }
    arm_last_orientation_ = arm_orientation_;
    arm_last_position_ = arm_position_;
    last_arm_twist_time_ = msg->header.stamp.toSec();
    arm_twist_flag_ = true;


}

void HybridForce::desired_state_callback(
        const cartesian_state_msgs::PoseTwistConstPtr &msg) {
    arm_desired_velocity_twist << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z,
            msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z;
}

void HybridForce::state_wrench_callback(
        const geometry_msgs::WrenchStampedConstPtr &msg) {
    Vector6d wrench_ft_frame;
    if (ft_arm_ready_) {
        wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
                msg->wrench.torque.y, msg->wrench.torque.z;

        //this force is represented in ft sensor link
        wrench_external_ << wrench_ft_frame;
    }
}

void HybridForce::desired_wrench_callback(const geometry_msgs::WrenchStampedConstPtr &msg) {
    if (ft_arm_ready_) {
        //this force is wrt task frame
        wrench_desired_threshold_
                << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
                msg->wrench.torque.y, msg->wrench.torque.z;

    }
}

//!-               COMMANDING THE ROBOT                  -!//
void HybridForce::send_commands_to_robot(const Vector6d &cmd) {

    geometry_msgs::Twist arm_twist_cmd;
    arm_twist_cmd.linear.x = cmd(0);
    arm_twist_cmd.linear.y = cmd(1);
    arm_twist_cmd.linear.z = cmd(2);
    arm_twist_cmd.angular.x = cmd(3);
    arm_twist_cmd.angular.y = cmd(4);
    arm_twist_cmd.angular.z = cmd(5);
    pub_arm_cmd_.publish(arm_twist_cmd);

}

//!-                    UTILIZATION                      -!//

bool HybridForce::get_rotation_matrix(Matrix6d &rotation_matrix,
                                      tf::TransformListener &listener,
                                      const std::string &from_frame,
                                      const std::string &to_frame) {
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
    catch (const tf::TransformException &x) {
        rotation_matrix.setZero();
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame);
        return false;
    }
    return true;
}

