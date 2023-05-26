#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include "robotiq_ft_sensor/sensor_accessor.h"

geometry_msgs::Vector3 mean(std::vector<geometry_msgs::Vector3> &vec);
geometry_msgs::Vector3 standard_deviation(std::vector<geometry_msgs::Vector3> &vec, geometry_msgs::Vector3 mean);
geometry_msgs::Vector3 set(double x, double y, double z);
void zero_sensor(robotiq_ft_sensor::sensor_accessor &srv, ros::ServiceClient &client);

#endif