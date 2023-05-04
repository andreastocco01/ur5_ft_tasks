#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>

void load_controller(ros::ServiceClient client, controller_manager_msgs::LoadController srv, char* controller) {
    srv.request.name = controller;
    if(client.call(srv)) ROS_INFO("LOAD %s", controller);
    else ROS_ERROR("FAILED to load %s", controller);
}

void switch_controllers(ros::ServiceClient client, controller_manager_msgs::SwitchController srv, char* start, char* stop) {
    srv.request.start_controllers.push_back(start);
    srv.request.stop_controllers.push_back(stop);
    srv.request.strictness = srv.request.STRICT;

    if(client.call(srv)) ROS_INFO("START %s STOP %s", start, stop);
    else ROS_ERROR("FAILED to switch controllers");

    srv.request.start_controllers.clear();
    srv.request.stop_controllers.clear();
}