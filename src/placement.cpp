#include <ros/ros.h>
#include <ros/topic.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>

using namespace moveit::planning_interface;

const std::string PLANNING_GROUP_ARM = "ur5_arm";

std::vector<geometry_msgs::PoseStamped> positions;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    positions.push_back(*msg);
}


int main(int argc, char** argv){

    // Initialize node and get istance
    ros::init(argc, argv, "placer");
    ros::NodeHandle node;

    // Starting Async rosSpin() for moveit
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
    ros::Subscriber pos_subscriber = node.subscribe("task_positions", 2, callback);

    // Wait for message in /task_positions topic
    ROS_INFO("Waiting for positions");

    while(positions.size() < 2);

    ROS_INFO("Positions received");

    // Move to first position
    positions[0].pose.position.z += 0.2;
    move_group.setPoseTarget(positions[0].pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // TODO: descend
    positions[0].pose.position.z -= 0.2;
    move_group.setPoseTarget(positions[0].pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Close gripper
    gripper_position.data = 110;
    gripper_publisher.publish(gripper_position);

    // Move to previous position
    positions[0].pose.position.z += 0.2;
    move_group.setPoseTarget(positions[0].pose);
    move_group.move();  // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Move to second position
    positions[1].pose.position.z += 0.2;
    move_group.setPoseTarget(positions[1].pose);
    move_group.move(); // Plan and execute movement waiting for success
    move_group.clearPoseTargets(); // Clear all targets specified

    // Open gripper
    gripper_position.data = 110;
    gripper_publisher.publish(gripper_position);
    
    return 0;
}