#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>

void load_controller(ros::ServiceClient client, controller_manager_msgs::LoadController srv, char* controller);
void switch_controllers(ros::ServiceClient client, controller_manager_msgs::SwitchController srv, char* start, char* stop);

#endif