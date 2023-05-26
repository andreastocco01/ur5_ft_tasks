#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include "robotiq_ft_sensor/sensor_accessor.h"

geometry_msgs::Vector3 mean(std::vector<geometry_msgs::Vector3> &vec) {
    geometry_msgs::Vector3 mean;
    mean.x = mean.y = mean.z = 0.0;
    for(int i = 0; i < vec.size(); i++) {
        mean.x += vec[i].x;
        mean.y += vec[i].y;
        mean.z += vec[i].z;
    }

    mean.x /= vec.size();
    mean.y /= vec.size();
    mean.z /= vec.size();

    return mean;
}

geometry_msgs::Vector3 standard_deviation(std::vector<geometry_msgs::Vector3> &vec, geometry_msgs::Vector3 mean) {
    geometry_msgs::Vector3 sd;
    sd.x = sd.y = sd.z = 0.0;
    for (int i = 0; i < vec.size(); i++) {
        sd.x += pow(vec[i].x - mean.x, 2);
        sd.y += pow(vec[i].y - mean.y, 2);
        sd.z += pow(vec[i].z - mean.z, 2);
    }

    sd.x = sqrt(sd.x / vec.size());
    sd.y = sqrt(sd.y / vec.size());
    sd.z = sqrt(sd.z / vec.size());

    return sd;
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