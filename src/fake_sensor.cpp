#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "fake_sensor");
    ros::NodeHandle node;

    ros::Rate rate(100);
    ros::Publisher publisher = node.advertise<geometry_msgs::WrenchStamped>("robotiq_ft_wrench", 512);
    geometry_msgs::WrenchStamped message;

    while (ros::ok()) {
        message.wrench.force.x = 10;
        message.wrench.force.y = 0;
        message.wrench.force.z = 0;
        publisher.publish(message);
        ROS_INFO("I sent: Fx[%f], Fy[%f], Fz[%f]", message.wrench.force.x, message.wrench.force.y, message.wrench.force.z);
        rate.sleep();
    }

    message.wrench.force.x = 0;
    message.wrench.force.y = 0;
    message.wrench.force.z = 0;
    publisher.publish(message);
    ROS_INFO("I sent: Fx[%f], Fy[%f], Fz[%f]", message.wrench.force.x, message.wrench.force.y, message.wrench.force.z);

    return 0;
}