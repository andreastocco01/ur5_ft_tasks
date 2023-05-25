#include <ros/ros.h>
#include "ur5_ft_tasks/controllers.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>

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
    ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");
    ros::shutdown();
}

geometry_msgs::Vector3 set(double x, double y, double z) {
    geometry_msgs::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z;

    return vector;
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

    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    controller_manager_msgs::LoadController load_srv;
    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);

    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    robotiq_ft_sensor::sensor_accessor srv;
    zero_sensor(srv, client);

    geometry_msgs::Twist move;
    move.linear = set(0, 0, 0);
    move.angular = set(0, 0, 0);

    while(force.z > -1) {
        move.linear = set(0, 0, -0.02);
        publisher.publish(move);
    }
    
    ros::Rate rate(72); // 5 secondi per fare un angolo giro
    double w = 0.25;
    double radius = 0.05;

    while(ros::ok && abs(force.x) < 7 && abs(force.y) < 7) {
        for(int angle = 0; angle < 360; angle++) {
            double rad = angle * M_PI / 180;
            double v = radius * w;
            if(force.z > -1) move.linear = set(v * sin(rad), v * cos(rad), -0.01);
            else move.linear = set(v * sin(rad), v * cos(rad), 0);
            if(abs(force.x) >= 7 || abs(force.y) >= 7) break;
            publisher.publish(move);
            ROS_INFO("%f, %f, %f", move.linear.x, move.linear.y, move.linear.z);
            rate.sleep();
        }
        move.linear = set(0, 0, 0);
        radius += 0.01;
    }
    mySigintHandler(SIGINT);
    return 0;
}