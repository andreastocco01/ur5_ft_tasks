#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt16.h>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include "ur5_ft_tasks/utils.h"
#include "ur5_ft_tasks/controllers.h"

using namespace moveit::planning_interface;

geometry_msgs::Vector3 force, torque;
ros::ServiceClient switch_client;
const std::string PLANNING_GROUP_ARM = "ur5_arm";
controller_manager_msgs::SwitchController switch_srv;

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "spiral");
    ros::NodeHandle node;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveGroupInterface move_group(PLANNING_GROUP_ARM);

    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    controller_manager_msgs::LoadController load_srv;
    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher gripper_publisher = node.advertise<std_msgs::UInt16>("/gripper_control/servo_debug", 1);

    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    robotiq_ft_sensor::sensor_accessor srv;
    zero_sensor(srv, client);

    geometry_msgs::Twist move;
    move.angular = set(0, 0, 0);
    double th = 6.5;
    bool first_time = true;
    geometry_msgs::PoseStamped height;

    // Init
    ros::Rate rate(30); // 5 secondi per fare un angolo giro
    double w = 0.25;
    double radius = 0.005;
    double angle = 0;

keep_contact:
    while(force.z > -8) {
        move.linear = set(0, 0, -0.015);
        publisher.publish(move);
    }
    move.linear = set(0, 0, 0);
    
    if(first_time) height = move_group.getCurrentPose("robotiq_ft_frame_id");
    first_time = false;

    geometry_msgs::PoseStamped current = move_group.getCurrentPose("robotiq_ft_frame_id");
    while(ros::ok && (height.pose.position.z - current.pose.position.z) < 0.02 && force.z <= -8) {
        double rad = angle * M_PI / 180;
        double v = radius * w;
        move.linear = set(v * sin(rad), v * cos(rad), 0);
        publisher.publish(move);
        radius += 0.00001;
        angle++;
        rate.sleep();
    }

    if(force.z > -8 && (height.pose.position.z - current.pose.position.z) < 0.02) {
        goto keep_contact;
    }
    
    move.linear = set(0, 0, 0);
    publisher.publish(move);
    std_msgs::UInt16 gripper_position;
    gripper_position.data = 180;
    gripper_publisher.publish(gripper_position);
    sleep(1);

    mySigintHandler(SIGINT);
    return 0;
}