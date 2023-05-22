#include <ros/ros.h>
#include <ur5_ft_tasks/controllers.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

ros::ServiceClient switch_client;
controller_manager_msgs::SwitchController switch_srv;

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

    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    geometry_msgs::Twist move;
    move.linear = set(0, 0, 0);
    move.angular = set(0, 0, 0);

    ros::Rate rate(72); // 5 secondi per fare un angolo giro
    double w = 0.25;

    for(double radius = 0.05; radius < 0.25; radius += 0.01) {
        ROS_WARN("Radius: %f", radius);
        for(int angle = 0; angle < 360; angle++) {
            double rad = angle * M_PI / 180;
            ROS_WARN("Angle: %d", angle);
            double v = radius * w;
            move.linear = set(v * sin(rad), v * cos(rad), 0);
            publisher.publish(move);
            ROS_INFO("%f, %f, %f", move.linear.x, move.linear.y, move.linear.z);
            rate.sleep();
        }
        move.linear = set(0, 0, 0);
    }
    mySigintHandler(SIGINT);
    return 0;
}