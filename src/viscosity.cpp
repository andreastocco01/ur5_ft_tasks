#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <ur5_ft_tasks/controllers.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ur5_ft_tasks/utils.h>
#include <signal.h>

geometry_msgs::Vector3 force, torque;
ros::ServiceClient switch_client;
controller_manager_msgs::SwitchController switch_srv;

void zero_sensor(robotiq_ft_sensor::sensor_accessor &srv, ros::ServiceClient &client) {
    srv.request.command_id = srv.request.COMMAND_SET_ZERO;

    if (client.call(srv)) {
        ROS_INFO("Zeroing: %s", srv.response.res.c_str());
    }
}

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    //ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

geometry_msgs::Vector3 set(double x, double y, double z) {
    geometry_msgs::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z;

    return vector;
}

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");

    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "viscosity");
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

    // move to home position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Move to HOME pose %s", success ? "SUCCESS" : "FAILED");
    move_group.move();

    // take current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose("robotiq_ft_frame_id"); // this is the end-effector!!!

    // go down
    geometry_msgs::Pose target_pose;
    target_pose.position.x = current_pose.pose.position.x;
    target_pose.position.y = current_pose.pose.position.y;
    target_pose.position.z = current_pose.pose.position.z - 0.177;
    target_pose.orientation = current_pose.pose.orientation;

    move_group.setPoseTarget(target_pose);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Down move %s", success ? "SUCCESS" : "FAILED");
    move_group.move();

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    controller_manager_msgs::LoadController load_srv;
    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    robotiq_ft_sensor::sensor_accessor srv;

    zero_sensor(srv, client);

    geometry_msgs::Twist move;
    geometry_msgs::Vector3 linear, angular;
    linear = set(0.0, 0.0, 0.0);
    angular = set(0.0, 0.0, 0.0);
    move.linear = linear;
    move.angular = angular;
    publisher.publish(move);

    double speed = 0.8;
    int count = 0;
    int times = 0;
    ros::Rate rate(70);
    std::vector<geometry_msgs::Vector3> vec;

    while(ros::ok()) {
        if(times == 2) break;
        if(count <= 100) angular = set(0, 0, -speed);
        else if (count > 100 && count <= 300) angular = set(0, 0, speed);
        else if (count > 300 && count < 400) angular = set(0, 0, -speed);
        else {
            count = 0;
            times++;
        }
        count++;
        move.angular = angular;
        publisher.publish(move);
        ros::spinOnce();
        vec.push_back(torque);
        rate.sleep();
    }

    ROS_INFO("Mean: %f\nStandard deviation: %f", mean(vec).z, standard_deviation(vec, mean(vec)).z);

    mySigintHandler(SIGINT);
    return 0;
}