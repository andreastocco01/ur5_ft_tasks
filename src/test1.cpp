#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 force;
geometry_msgs::Vector3 torque;

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    force = msg->wrench.force;
    torque = msg->wrench.torque;
    ROS_INFO("I heard: fx[%f], fy[%f], fz[%f], tx[%f], ty[%f], tz[%f]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "test1");
    ros::NodeHandle node;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface current_scene;

    //ros::ServiceClient client = n.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);

    // 1. Move to home position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move to HOME pose %s", success ? "SUCCESS" : "FAILED");

    move_group.move();

    sleep(80);

    int direction_x = 0;
    int direction_y = 0;
    int direction_z = 0;
    const double variation = 0.1;

    while(ros::ok()) {
        geometry_msgs::PoseStamped current_pose;
        current_pose = move_group.getCurrentPose("robotiq_ft_frame_id");
        geometry_msgs::PoseStamped target_pose = current_pose;

        if(force.x > 10) direction_x = -1;
        else if(force.x < -10) direction_x = 1;
        else direction_x = 0;

        if(force.y > 10) direction_y = -1;
        else if(force.y < -10) direction_y = 1;
        else direction_y = 0;

        if(force.z > 10) direction_z = -1;
        else if(force.z < -10) direction_z = 1;
        else direction_z = 0;

        if (direction_x != 0 || direction_y != 0 || direction_z != 0) { // if at least one of the direction isn't zero, move
            target_pose.pose.position.x += direction_x * variation;
            target_pose.pose.position.y += direction_y * variation;
            target_pose.pose.position.z += direction_z * variation;
            move_group.setPoseTarget(target_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Target move %s", success ? "SUCCESS" : "FAILED");
            move_group.move();
        }
        ros::spinOnce();
    }
    return 0;
}