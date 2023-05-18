#include <ros/ros.h>
#include <ros/topic.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>

namespace moveit_planner = moveit::planning_interface;

const std::string PLANNING_GROUP_ARM = "ur5_arm";


int main(int argc, char** argv){

    // Gripper position message
    std_msgs::UInt16 gripper_position;

    // Initialize node and get istance
    ros::init(argc, argv, "placer");
    ros::NodeHandle node;

    // Init moveit
    moveit_planner::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
    moveit_planner::MoveGroupInterface::Plan plan;
    moveit_planner::PlanningSceneInterface scene;

    // Setup topics
    ros::Publisher gripper_publisher = node.advertise<std_msgs::UInt16>("/gripper_control/servo_debug", 1);
    //ros::Subscriber pos_subscriber = node.subscribe("task_positions");

    // Wait for message in /task_positions topic
    geometry_msgs::PoseStampedConstPtr positionOne = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("task_positions");
    // DEBUG: print position
    ROS_INFO("Ricevuta posizione %f, %f, %f", positionOne->pose.position.x, positionOne->pose.position.y, positionOne->pose.position.z);

    // Wait for message in /task_positions topic
    geometry_msgs::PoseStampedConstPtr positionTwo = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("task_positions");
    // DEBUG: print position
    ROS_INFO("Ricevuta posizione %f, %f, %f", positionTwo->pose.position.x, positionTwo->pose.position.y, positionTwo->pose.position.z);


    // Starting Async rosSpin() for moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //  Move to home position
    move_group.setNamedTarget("home");
    move_group.move();
    move_group.clearPoseTargets(); // Clear all targets specified


    // Move robot to first position
    move_group.setPoseTarget(positionOne->pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // TODO: descend

    // Close gripper
    gripper_position.data = 110;
    gripper_publisher.publish(gripper_position);

    // Move robot to second position
    move_group.setPoseTarget(positionOne->pose);
    move_group.move(); // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Open gripper
    gripper_position.data = 110;
    gripper_publisher.publish(gripper_position);
    
    return 0;
}