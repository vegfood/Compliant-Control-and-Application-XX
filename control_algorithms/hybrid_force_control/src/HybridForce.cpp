#include "hybrid_force_control/HybridForce.h"

#include <utility>

HybridForce::HybridForce(ros::NodeHandle &n,
                       double frequency,
                       const std::string& topic_arm_state,
                       const std::string& topic_arm_command,
                       const std::string& topic_wrench_state,
                       const std::string& topic_wrench_desired,
                       const std::string& topic_desired_state,
                       std::vector<double> M,
                       std::vector<double> D,
                       std::vector<double> K,
                       std::vector<double> desired_pose,
                       std::string base_link,
                       const std::string& end_link,
                       std::string interface_type,
                       double arm_max_vel,
                       double arm_max_acc) :
        nh_(n), loop_rate_(frequency),
        M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
        arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
        base_link_(std::move(base_link)), end_link_(end_link), control_frame_(end_link), interface_type_(std::move(interface_type)){

    //* Subscribers
    sub_arm_state_           = nh_.subscribe(topic_arm_state, 5,
                                             &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
                                             &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_desired_        = nh_.subscribe(topic_wrench_desired, 5,
                                               &Admittance::desired_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_state_        = nh_.subscribe(topic_desired_state, 5,
                                              &Admittance::desired_state_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    //* Publishers
    if (interface_type_ == "velocity"){
        pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

    }
    else {
        pub_arm_cmd_ = nh_.advertise<geometry_msgs::Pose>(topic_arm_command, 5);

    }

    // initializing the class variables
    arm_position_.setZero();
    arm_twist_.setZero();
    wrench_external_.setZero();
    wrench_desired_threshold_.setZero();

    desired_pose_position_ << desired_pose_.topRows(3);
    desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();
    arm_desired_velocity_twist.setZero();
    arm_desired_acceleration.setZero();

    arm_desired_velocity_twist_adm_.setZero();
    arm_desired_acceleration_adm_.setZero();


    while (nh_.ok() && !arm_position_(0)) {
        ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
        ros::spinOnce();
        loop_rate_.sleep();
    }

    // Init integrator
    delta_x_pre.setZero();
    dot_delta_x_pre.setZero();

    //init ft sensor frame flag
    ft_arm_ready_ = false;

    wait_for_transformations();
}

//!-                   INITIALIZATION                    -!//

void Hybridforce::wait_for_transformations() {
    tf::TransformListener listener;
    Matrix6d rot_matrix;
    while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
    ft_arm_ready_ = true;
    ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void Hybridforce::run() {

    ROS_INFO("Running the admittance control loop .................");

    while (nh_.ok()) {
        if (interface_type_ == "velocity"){
            Vector6d cmd = compute_admittance_velocity_interface();
            send_commands_to_robot(cmd);

        }
        else{
//        Vector7d cmd = compute_admittance_position_interface();
            Vector7d cmd = compute_admittance_simplified_position_interface();
            send_commands_to_robot(cmd);
        }

        ros::spinOnce();
        loop_rate_.sleep();
    }
}

//!-                基于速度接口的混合力控实现                  -!//
Vector6d Hybridforce::compute_hybrid_force_velocity_interface() {
    //基于内环为速度的导纳控制器：K_d(v_d - v_e) + K_p(x_d - x_e) = h_e, v_r(v_e) = v_d + K_d.inverse()*(K_p(x_d - x_e) - h_e)
    //D，K矩阵默认为定义在控制坐标系下的对角矩阵
    //为了简化处理，选择末端坐标系作为控制坐标系,即control frame 等于 end_frame

    // D_base = rot_control_base * D * rot_control_base.transpose()
    // K_base = rot_control_base * K * rot_control_base.transpose()
    Eigen::Matrix<double, 6, 6> D_base = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> K_base = Eigen::Matrix<double, 6, 6>::Zero();

    //求解控制坐标系在基坐标系下的旋转矩阵
    Matrix6d rot_control_base;
    get_rotation_matrix(rot_control_base, tf_listener_, control_frame_, base_link_);
    //求解基坐标系下的各项矩阵
    D_base = rot_control_base * D_ * rot_control_base.transpose();
    K_base = rot_control_base * K_ * rot_control_base.transpose();

    //基坐标系下：计算位姿误差
    // Translation error，x_d - x_e
    Vector6d error;
    error.topRows(3) = desired_pose_position_ - arm_position_;
    if(arm_orientation_.coeffs().dot(desired_pose_orientation_.coeffs()) < 0.0)
    {
        arm_orientation_.coeffs() << -arm_orientation_.coeffs();
    }

    //期望坐标系相对于基坐标系的旋转矩阵, 参考https://github.com/frankaemika/libfranka/blob/main/examples/cartesian_impedance_control.cpp
    Matrix3d R_desired_base = desired_pose_orientation_.toRotationMatrix();
    Eigen::Quaterniond quat_rot_err (desired_pose_orientation_.inverse() * arm_orientation_);
    if(quat_rot_err.coeffs().norm() > 1e-3)
    {
        quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
    }
    //表示在期望姿态坐标系下的相对姿态误差
    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);

    error.bottomRows(3) << - R_desired_base * err_arm_des_orient.axis() * err_arm_des_orient.angle();

    //求解基坐标系下的交互力, 交互力与测量受到的外力方向相反, 希望的交互力默认在末端坐标系下表示，故两者之差表示合交互力
    Matrix6d rot_ft_base;
    get_rotation_matrix(rot_ft_base, tf_listener_, end_link_, base_link_);
    Vector6d F_base = rot_ft_base * (-wrench_external_ - wrench_desired_threshold_);

    //求解速度环的参考速度v_r =  v_d + K_d.inverse()*(K_p(x_d - x_e) - h_e)

    arm_desired_velocity_twist_adm_ = arm_desired_velocity_twist + D_base.inverse() * (K_base * error - F_base);

    return arm_desired_velocity_twist_adm_;

}


//!-                     CALLBACKS                       -!//

void HybridForce::state_arm_callback(
        const cartesian_state_msgs::PoseTwistConstPtr& msg) {
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

void HybridForce::desired_state_callback(
        const cartesian_state_msgs::PoseTwistConstPtr& msg) {
    desired_pose_position_ <<  msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    desired_pose_orientation_.coeffs() <<  msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w;

    arm_desired_velocity_twist << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z,
            msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z;
}

void HybridForce::state_wrench_callback(
        const geometry_msgs::WrenchStampedConstPtr& msg) {
    Vector6d wrench_ft_frame;
    if (ft_arm_ready_) {
        wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,msg->wrench.force.z,msg->wrench.torque.x,
                msg->wrench.torque.y,msg->wrench.torque.z;

        //this force is represented in ft sensor link
        wrench_external_ << wrench_ft_frame;
    }
}

void HybridForce::desired_wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    if (ft_arm_ready_) {
        //this force is wrt ft sensor link
        wrench_desired_threshold_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
                msg->wrench.torque.y,msg->wrench.torque.z;

    }
}

//!-               COMMANDING THE ROBOT                  -!//
void HybridForce::send_commands_to_robot(const Vector6d & cmd) {

    geometry_msgs::Twist arm_twist_cmd;
    arm_twist_cmd.linear.x  = cmd(0);
    arm_twist_cmd.linear.y  = cmd(1);
    arm_twist_cmd.linear.z  = cmd(2);
    arm_twist_cmd.angular.x = cmd(3);
    arm_twist_cmd.angular.y = cmd(4);
    arm_twist_cmd.angular.z = cmd(5);
    pub_arm_cmd_.publish(arm_twist_cmd);

}

//!-                    UTILIZATION                      -!//

bool HybridForce::get_rotation_matrix(Matrix6d & rotation_matrix,
                                     tf::TransformListener & listener,
                                     const std::string& from_frame,
                                     const std::string& to_frame) {
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
    catch (const tf::TransformException & x) {
        rotation_matrix.setZero();
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
        return false;
    }
    return true;
}

