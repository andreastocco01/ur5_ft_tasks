#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <signal.h>

geometry_msgs::Vector3 force, torque;
geometry_msgs::Twist move;
ros::Publisher publisher;

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

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    move.linear.x = 0;
    move.linear.y = 0;
    move.linear.z = 0;
    publisher.publish(move);
    ROS_INFO("Stopped!");

    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "viscosity");
    ros::NodeHandle node;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);

    robotiq_ft_sensor::sensor_accessor srv;

    zero_sensor(srv, client);

    geometry_msgs::Vector3 linear, angular;
    linear = set(0.0, 0.0, 0.0);
    angular = set(0.0, 0.0, 0.0);
    move.linear = linear;
    move.angular = angular;
    publisher.publish(move);

    double speed = 0.05;
    int count = 0;
    ros::Rate rate(10);

    while(ros::ok()) {
        ROS_INFO("%d", count);
        if(count <= 100) linear = set(speed, 0, 0);
        else if (count > 100 && count < 200) linear = set(-speed, 0, 0);
        else count = 0;
        count++;
        move.linear = linear;
        publisher.publish(move);
        //ros::spinOnce();
        rate.sleep();
    }
    return 0;
}