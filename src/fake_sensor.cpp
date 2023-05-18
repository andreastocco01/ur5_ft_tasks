#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

geometry_msgs::Vector3 force, torque;

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
}

geometry_msgs::Vector3 set(double x, double y, double z) {
    geometry_msgs::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z;

    return vector;
}

void zero_sensor(robotiq_ft_sensor::sensor_accessor &srv, ros::ServiceClient &client) {
    srv.request.command_id = srv.request.COMMAND_SET_ZERO;

    if (client.call(srv)) {
        ROS_INFO("Zeroing: %s", srv.response.res.c_str());
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "fake_sensor");
    ros::NodeHandle node;

    ros::Rate rate(100);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);

    robotiq_ft_sensor::sensor_accessor srv;

    zero_sensor(srv, client);
    geometry_msgs::Twist twist;
    twist.linear = set(0,0,0);
    twist.angular = set(0,0,0);
    ROS_INFO("I heard: fz[%f]", force.z);
    while (ros::ok() && !(force.z < -1)) {
        ROS_INFO("I heard: fz[%f]", force.z);
        twist.linear = set(0,0,-0.007);
        publisher.publish(twist);
        ros::spinOnce();
    }

    twist.linear = set(0,0,0.007);
    publisher.publish(twist);
    sleep(2);
    twist.linear = set(0,0,0);
    publisher.publish(twist);
    return 0;
}