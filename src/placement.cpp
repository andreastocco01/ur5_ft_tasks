#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include "ur5_ft_tasks/utils.h"
#include "ur5_ft_tasks/controllers.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

using namespace moveit::planning_interface;

geometry_msgs::Vector3 force, torque;
const std::string PLANNING_GROUP_ARM = "ur5_arm";
controller_manager_msgs::SwitchController switch_srv;
ros::ServiceClient switch_client;

std::vector<geometry_msgs::PoseStamped> positions;

void callback_pos(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    positions.push_back(*msg);
}

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    //ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");
    ros::shutdown();
}

int main(int argc, char** argv){

    // Initialize node and get istance
    ros::init(argc, argv, "placer");
    ros::NodeHandle node;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    // Starting Async spin() for moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Gripper position message
    std_msgs::UInt16 gripper_position;

    // Init moveit
    MoveGroupInterface move_group(PLANNING_GROUP_ARM);
    MoveGroupInterface::Plan plan;
    PlanningSceneInterface scene;

    // Setup topics
    ros::Publisher gripper_publisher = node.advertise<std_msgs::UInt16>("/gripper_control/servo_debug", 1);
    ros::Subscriber pos_subscriber = node.subscribe("task_positions", 2, callback_pos);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::LoadController load_srv;

    // Wait for messages in /task_positions topic
    ROS_INFO("Waiting for positions");

    while(positions.size() < 2);

    ROS_INFO("Positions received");

    robotiq_ft_sensor::sensor_accessor srv;
    zero_sensor(srv, client);

    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    // Close gripper
    gripper_position.data = 30;
    gripper_publisher.publish(gripper_position);
    sleep(1);

    // Down until table_contact and save position
    geometry_msgs::Twist move;
    move.angular = set(0, 0, 0);
    while(force.z > -8) {
        move.linear = set(0, 0, -0.015);
        publisher.publish(move);
    }
    move.linear = set(0, 0, 0);
    publisher.publish(move);
    geometry_msgs::PoseStamped table_height = move_group.getCurrentPose("robotiq_ft_frame_id");

    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");

    // Move to first position
    positions[0].pose.position.z += 0.1;
    move_group.setPoseTarget(positions[0].pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    // Down until object contact and save
    while(force.z > -8) {
        move.linear = set(0, 0, -0.015);
        publisher.publish(move);
    }
    move.linear = set(0, 0, 0);
    publisher.publish(move);
    geometry_msgs::PoseStamped object_height = move_group.getCurrentPose("robotiq_ft_frame_id");

    switch_controllers(switch_client, switch_srv, (char*)"scaled_pos_joint_traj_controller", (char*)"twist_controller");

    // Little move up
    object_height.pose.position.z += 0.01;
    move_group.setPoseTarget(object_height.pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Open gripper
    gripper_position.data = 180;
    gripper_publisher.publish(gripper_position);
    sleep(1);

    // Down to half object height
    object_height.pose.position.z -= 0.01;
    object_height.pose.position.z -= ((object_height.pose.position.z - table_height.pose.position.z) / 2) + 0.015;
    move_group.setPoseTarget(object_height.pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Close gripper
    gripper_position.data = 0;
    gripper_publisher.publish(gripper_position);
    sleep(1);

    // Move to first position
    positions[0].pose.position.z;
    move_group.setPoseTarget(positions[0].pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Move to second position
    positions[1].pose.position.z += 0.1;
    move_group.setPoseTarget(positions[1].pose);
    move_group.move(); // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    mySigintHandler(SIGINT);
    return 0;
}