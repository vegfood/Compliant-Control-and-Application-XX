/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:30 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:30 
 * @Licence: MIT Licence
 */
#include "ros/ros.h"
#include "admittance/Admittance.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_node");

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

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> desired_pose;
    
    double arm_max_vel;
    double arm_max_acc;


    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    if (!nh.getParam("topic_wrench_desired", topic_wrench_desired)) { ROS_ERROR("Couldn't retrieve the topic name for the desired wrench."); return -1; }
    if (!nh.getParam("topic_desired_state", topic_desired_state)) { ROS_ERROR("Couldn't retrieve the topic name for the desired arm state."); return -1; }

    // ADMITTANCE PARAMETERS
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; }
    if (!nh.getParam("interface_type", interface_type)) { ROS_ERROR("Couldn't retrieve the interface_type."); return -1; }

    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    // Constructing the controller
    Admittance admittance(
        nh,
        frequency,
        topic_arm_state,
        topic_arm_command,
        topic_wrench_state,
        topic_wrench_desired,
        topic_desired_state,
        M, D, K, desired_pose,
        base_link,
        end_link,
        interface_type,
        arm_max_vel,
        arm_max_acc);

    // Running the controller
    admittance.run();

    return 0;
}