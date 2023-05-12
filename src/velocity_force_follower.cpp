#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <geometry_msgs/Twist.h>
#include <ur5_ft_tasks/controllers.h>
#include <stdlib.h>
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

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");

    ros::shutdown();
}

void feedback(geometry_msgs::Twist move) {
    ros::Time startTime = ros::Time::now();
    while((ros::Time::now() - startTime).toSec() < 0.1) {
        move.linear = set(0, 0, 0.4);
    }
    startTime = ros::Time::now();
    while((ros::Time::now() - startTime).toSec() < 0.1) {
        move.linear = set(0, 0, -0.4);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_force_follower");
    ros::NodeHandle node;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface current_scene;

    // Move to home position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Move to HOME pose %s", success ? "SUCCESS" : "FAILED");
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

    int dx = 0;
    int dy = 0;
    int dz = 0;

    double speedx = 0.0;
    double speedy = 0.0;
    double speedz = 0.0;

    ros::Rate rate(10);
    int alpha = 300;
    int threshold = 8;
    std::vector<geometry_msgs::PoseStamped> positions;

    while(ros::ok()) {
        if(force.x > threshold) dx = 1;
        else if(force.x < -threshold) dx = -1;
        else dx = 0;

        if(force.y > threshold) dy = -1;
        else if(force.y < -threshold) dy = 1;
        else dy = 0;

        if(force.z > threshold) dz = -1;
        else if(force.z < -threshold) dz = 1;
        else dz = 0;

        if(dx != 0) speedx = force.x / alpha;
        else if (dy != 0) speedy = force.y / alpha;
        else if (dz != 0) speedz = force.z / alpha;
        else {
            speedx = 0.0;
            speedy = 0.0;
            speedz = 0.0;
        }

        if(std::abs(torque.z) > 20) {
            positions.push_back(move_group.getCurrentPose("robotiq_ft_frame_id"));
            feedback(move);
        }

        // linear is (0, 0, 0) if force < threshold or a position is saved
        linear = set(dx * std::abs(speedx), dy * std::abs(speedy), dz * std::abs(speedz));
        move.linear = linear;

        publisher.publish(move);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}