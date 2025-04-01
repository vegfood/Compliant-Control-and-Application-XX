#include "ros/ros.h"
#include "hybrid_force_control//HybridForce.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hybrid_force_node");

    ros::NodeHandle nh;
    double frequency = 100.0;

    // Parameters
    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_wrench_state;
    std::string topic_wrench_desired;
    std::string topic_desired_state;

    std::string base_link;
    std::string end_link;
    std::string interface_type;

    std::vector<double> K_i_v;
    std::vector<double> K_i_lambda;
    std::vector<double> K_p_lambda;
    std::vector<double> S_v;
    std::vector<double> S_f;
    std::vector<double> W_v;
    std::vector<double> W_f;
    std::vector<double> K_env;

    std::vector<double> desired_pose;


    double arm_max_vel;
    double arm_max_acc;


    // LOADING PARAMETERS FROM THE ROS SERVER

    // Topic names
    if (!nh.getParam("topic_arm_state", topic_arm_state)) {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
        return -1;
    }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
        return -1;
    }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) {
        ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor.");
        return -1;
    }
    if (!nh.getParam("topic_wrench_desired", topic_wrench_desired)) {
        ROS_ERROR("Couldn't retrieve the topic name for the desired wrench.");
        return -1;
    }
    if (!nh.getParam("topic_desired_state", topic_desired_state)) {
        ROS_ERROR("Couldn't retrieve the topic name for the desired arm state.");
        return -1;
    }

    // ADMITTANCE PARAMETERS
    if (!nh.getParam("K_i_v", K_i_v)) {
        ROS_ERROR("Couldn't retrieve the desired integral gain of velocity.");
        return -1;
    }
    if (!nh.getParam("K_i_lambda", K_i_lambda)) {
        ROS_ERROR("Couldn't retrieve the desired integral gain of force.");
        return -1;
    }
    if (!nh.getParam("K_p_lambda", K_p_lambda)) {
        ROS_ERROR("Couldn't retrieve the desired proportional gain of force.");
        return -1;
    }
    if (!nh.getParam("S_v", S_v)) {
        ROS_ERROR("Couldn't retrieve the selection matrix of velocity.");
        return -1;
    }
    if (!nh.getParam("S_f", S_f)) {
        ROS_ERROR("Couldn't retrieve the selection matrix of force.");
        return -1;
    }
    if (!nh.getParam("W_v", W_v)) {
        ROS_ERROR("Couldn't retrieve the weighting matrix of velocity.");
        return -1;
    }
    if (!nh.getParam("W_f", W_f)) {
        ROS_ERROR("Couldn't retrieve the weighting matrix of force.");
        return -1;
    }
    if (!nh.getParam("K_env", K_env)) {
        ROS_ERROR("Couldn't retrieve the stiffness matrix of environment.");
        return -1;
    }
    if (!nh.getParam("base_link", base_link)) {
        ROS_ERROR("Couldn't retrieve the base_link.");
        return -1;
    }
    if (!nh.getParam("end_link", end_link)) {
        ROS_ERROR("Couldn't retrieve the end_link.");
        return -1;
    }
    if (!nh.getParam("interface_type", interface_type)) {
        ROS_ERROR("Couldn't retrieve the interface_type.");
        return -1;
    }
    if (!nh.getParam("desired_pose", desired_pose)) {
        ROS_ERROR("Couldn't retrieve the desired pose of the spring.");
        return -1;
    }

    if (!nh.getParam("arm_max_vel", arm_max_vel)) {
        ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
        return -1;
    }
    if (!nh.getParam("arm_max_acc", arm_max_acc)) {
        ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
        return -1;
    }

    // Constructing the controller
    HybridForce hybridForce(
            nh,
            frequency,
            topic_arm_state,
            topic_arm_command,
            topic_wrench_state,
            topic_wrench_desired,
            topic_desired_state,
            K_i_v,
            K_i_lambda,
            K_p_lambda,
            S_v,
            S_f,
            W_v,
            W_f,
            K_env,
            desired_pose,
            base_link,
            end_link,
            interface_type,
            arm_max_vel,
            arm_max_acc);

    // Running the controller
    hybridForce.run();

    return 0;
}