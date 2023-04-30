#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Vector3 force, torque;

void zero_sensor(robotiq_ft_sensor::sensor_accessor &srv, ros::ServiceClient &client) {
    srv.request.command_id = srv.request.COMMAND_SET_ZERO;

    if (client.call(srv)) {
        ROS_INFO("ret: %s", srv.response.res.c_str());
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "test2");
    ros::NodeHandle node;

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);

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

    double speed = 0.0;

    while(ros::ok()) {
        if(force.x > 8) dx = 1;
        else if(force.x < -8) dx = -1;
        else dx = 0;

        if(force.y > 8) dy = -1;
        else if(force.y < -8) dy = 1;
        else dy = 0;

        if(force.z > 8) dz = -1;
        else if(force.z < -8) dz = 1;
        else dz = 0;

        if (dx != 0 || dy != 0 || dz != 0) speed = 0.1; // if at least one of the direction isn't zero, move
        else speed = 0.0;

        linear = set(dx * speed, dy * speed, dz * speed);
        move.linear = linear;

        publisher.publish(move);

        ros::spinOnce();
    }
    return 0;
}