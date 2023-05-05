#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 mean(std::vector<geometry_msgs::Vector3> &vec);
geometry_msgs::Vector3 standard_deviation(std::vector<geometry_msgs::Vector3> &vec, geometry_msgs::Vector3 mean);

#endif