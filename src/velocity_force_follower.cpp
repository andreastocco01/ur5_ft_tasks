#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>
#include "ur5_ft_tasks/controllers.h"
#include "ur5_ft_tasks/utils.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

geometry_msgs::Vector3 force, torque;
ros::ServiceClient switch_client;
controller_manager_msgs::SwitchController switch_srv;

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    //ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");
    ros::shutdown();
}

void feedback(ros::Publisher &publisher) {
    std_msgs::UInt16 x;
    x.data = 120;
    publisher.publish(x);
    ros::Duration(0.5).sleep();
    x.data = 180;
    publisher.publish(x);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_force_follower");
    ros::NodeHandle node;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface current_scene;

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    ros::Publisher pos_publisher = node.advertise<geometry_msgs::PoseStamped>("task_positions", 2);
    ros::Publisher gripper_publisher = node.advertise<std_msgs::UInt16>("/gripper_control/servo_debug", 1);
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    // 1. move to home position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Move to HOME pose %s", success ? "SUCCESS" : "FAILED");
    move_group.move();

    controller_manager_msgs::LoadController load_srv;
    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    robotiq_ft_sensor::sensor_accessor srv;

    zero_sensor(srv, client);

    std_msgs::UInt16 init;
    init.data = 180;
    gripper_publisher.publish(init);

    geometry_msgs::Twist move;
    geometry_msgs::Vector3 linear, angular;
    linear = set(0.0, 0.0, 0.0);
    angular = set(0.0, 0.0, 0.0);
    move.linear = linear;
    move.angular = angular;
    publisher.publish(move);

    ros::Rate rate(100);
    int alpha = 200;
    int threshold = 5;
    std::vector<geometry_msgs::PoseStamped> positions;
    std::vector<geometry_msgs::Vector3> forces;

    while(ros::ok() && positions.size() < 3) {
        if(forces.size() < 20) {
            forces.push_back(force);
        } else {
            forces.erase(forces.begin());
            forces.push_back(force);
            geometry_msgs::Vector3 current = mean(forces);
            double module = sqrt(pow(current.x, 2) + pow(current.y, 2) + pow(current.z, 2));
            if(module > threshold) {
                linear = set(current.x/alpha, -current.y/alpha, -current.z/alpha);
            } else {
                linear = set(0, 0, 0);
            }

            if(std::abs(torque.z) > 1) {
                positions.push_back(move_group.getCurrentPose("robotiq_ft_frame_id"));
                feedback(gripper_publisher);
                ROS_INFO("Acquired position");
            }

            // linear is (0, 0, 0) if force < threshold or a position is saved
            move.linear = linear;

            publisher.publish(move);
        }

        ros::spinOnce();
        rate.sleep();
    }

    pos_publisher.publish(positions[0]);
    pos_publisher.publish(positions[1]);

    mySigintHandler(SIGINT);
    return 0;
}