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

geometry_msgs::Vector3 force;
geometry_msgs::Vector3 torque;

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    //ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "test2");
    ros::NodeHandle node;

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);

    robotiq_ft_sensor::sensor_accessor srv;

    geometry_msgs::Twist move;

    int direction_x = 0;
    int direction_y = 0;
    int direction_z = 0;

    ros::Rate rate(10);

    while(ros::ok()) {

        double speed = 0.00;
        move.linear.x = direction_x * speed;
        move.linear.y = direction_y * speed;
        move.linear.z = direction_z * speed;

        publisher.publish(move);

        if(force.x > 4) direction_x = 1;
        else if(force.x < -4) direction_x = -1;
        else direction_x = 0;

        if(force.y > 4) direction_y = -1;
        else if(force.y < -4) direction_y = 1;
        else direction_y = 0;

        if(force.z > 10) direction_z = -1;
        else if(force.z < -10) direction_z = 1;
        else direction_z = 0;

        if (direction_x != 0 || direction_y != 0 || direction_z != 0) { // if at least one of the direction isn't zero, move

            speed = 0.1;

            move.linear.x = direction_x * speed;
            move.linear.y = direction_y * speed;
            move.linear.z = direction_z * speed;

            publisher.publish(move);

            // zero the sensor after every movement
            /*srv.request.command_id = srv.request.COMMAND_SET_ZERO;

            if (client.call(srv)) {
                ROS_INFO("ret: %s", srv.response.res.c_str());
            }*/
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}