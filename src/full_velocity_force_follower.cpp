#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "ur5_ft_tasks/controllers.h"
#include "ur5_ft_tasks/utils.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

#include <Eigen/Dense>

geometry_msgs::Vector3 force, torque;
ros::ServiceClient switch_client;
controller_manager_msgs::SwitchController switch_srv;

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

void feedback(ros::Publisher &publisher) {
    std_msgs::UInt16 x;
    x.data = 120;
    publisher.publish(x);
    ros::Duration(0.5).sleep();
    x.data = 180;
    publisher.publish(x);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_force_follower");
    ros::NodeHandle node;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

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

    ros::ServiceClient client = node.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ros::Subscriber subscriber = node.subscribe("robotiq_ft_wrench", 100, callback);
    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    ros::Publisher pos_publisher = node.advertise<geometry_msgs::PoseStamped>("task_positions", 2);
    ros::Publisher gripper_publisher = node.advertise<std_msgs::UInt16>("/gripper_control/servo_debug", 1);
    ros::ServiceClient load_client = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    switch_client = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    tf::TransformListener listener;

    // 1. move to home position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Move to HOME pose %s", success ? "SUCCESS" : "FAILED");
    move_group.move();

    controller_manager_msgs::LoadController load_srv;
    load_controller(load_client, load_srv, (char*)"twist_controller");
    switch_controllers(switch_client, switch_srv, (char*)"twist_controller", (char*)"scaled_pos_joint_traj_controller");

    robotiq_ft_sensor::sensor_accessor srv;

    zero_sensor(srv, client);

    std_msgs::UInt16 init;
    init.data = 180;
    gripper_publisher.publish(init);

    geometry_msgs::Twist move;
    geometry_msgs::Vector3 linear, angular;
    linear = set(0.0, 0.0, 0.0);
    angular = set(0.0, 0.0, 0.0);
    move.linear = linear;
    move.angular = angular;
    publisher.publish(move);

    ros::Rate rate(100);
    int alpha = 175;
    int threshold = 5;
    std::vector<geometry_msgs::PoseStamped> positions;
    std::vector<geometry_msgs::Vector3> forces;
    std::vector<geometry_msgs::Vector3> torques;
    tf::StampedTransform transform;

    while(ros::ok() && positions.size() < 3) {
        torque.x = torque.y = 0;

        ros::Time mytime = ros::Time::now();

        try {
             listener.lookupTransform("/base", "/robotiq_ft_frame_id",  
                                     ros::Time(0), transform);
         } catch (tf::TransformException ex) {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
        }

        tf::Vector3 origin;
        origin.setX(0);
        origin.setY(0);
        origin.setZ(0);
        transform.setOrigin(origin);



        if(forces.size() < 20 && torques.size() < 20) {
            forces.push_back(force);
            torques.push_back(torque);
        } else {
            forces.erase(forces.begin());
            torques.erase(torques.begin());
            forces.push_back(force);
            torques.push_back(torque);
            geometry_msgs::Vector3 fcurrent = mean(forces);
            geometry_msgs::Vector3 tcurrent = mean(torques);


            //geometry_msgs::Vector3Stamped fcurrent_stamped;
            //fcurrent_stamped.header.stamp = ros::Time::now();
            //fcurrent_stamped.header.frame_id = "/robotiq_ft_frame_id";
            //fcurrent_stamped.vector = fcurrent;
            // geometry_msgs::Vector3Stamped tcurrent_stamped;
            // tcurrent_stamped.header.stamp = ros::Time::now();
            // tcurrent_stamped.header.frame_id = "/robotiq_ft_frame_id";
            // tcurrent_stamped.vector = tcurrent;


            // Express force wrt base
            //geometry_msgs::Vector3Stamped fcurrent_base;
            //geometry_msgs::Vector3Stamped tcurrent_base;
            //tf::Transformer::transformVector(std::string("/base"), fcurrent_stamped, fcurrent_base);
            //tf::Transformer::transformVector(std::string("/base"), tcurrent_stamped, tcurrent_base);


            ros::Time now = ros::Time::now();
            geometry_msgs::PointStamped fcurrent_base;
            geometry_msgs::PointStamped fcurrent_stamped;
            fcurrent_stamped.header.stamp = now;
            fcurrent_stamped.header.frame_id = "/robotiq_ft_frame_id";
            fcurrent_stamped.point.x = fcurrent.x;
            fcurrent_stamped.point.y = fcurrent.y;
            fcurrent_stamped.point.z = fcurrent.z;


            std::cout << "waiting TF...\n";
            //listener.waitForTransform("/base", "/robotiq_ft_frame_id", now, ros::Duration(3.0));
            //listener.transformPoint(std::string("/base"), fcurrent_stamped, fcurrent_base);

            tf::Vector3 fcurrent_tfvector;
            fcurrent_tfvector.setX(fcurrent.x);
            fcurrent_tfvector.setY(fcurrent.y);
            fcurrent_tfvector.setZ(fcurrent.z);

            tf::Vector3 fcurrent_tfvector_out = transform * fcurrent_tfvector;

            fcurrent_base.point.x = double(fcurrent_tfvector_out.x());
            fcurrent_base.point.y= double(fcurrent_tfvector_out.y());
            fcurrent_base.point.z = double(fcurrent_tfvector_out.z());

            std::cout << fcurrent_base << std::endl;
            std::cout << fcurrent_stamped << std::endl;




            double fmodule = sqrt(pow(fcurrent.x, 2) + pow(fcurrent.y, 2) + pow(fcurrent.z, 2));
            double tmodule = sqrt(pow(tcurrent.x, 2) + pow(tcurrent.y, 2) + pow(tcurrent.z, 2));
            if(fmodule > threshold && tmodule > 0.1) {
                linear = set(fcurrent_base.point.x/alpha, fcurrent_base.point.y/alpha, fcurrent_base.point.z/alpha);
                angular = set(tcurrent.x, -tcurrent.y, -tcurrent.z);
            } else if (tmodule > 0.1){
                angular = set(tcurrent.x, -tcurrent.y, -tcurrent.z);
                linear = set(0, 0, 0);
            } else if (fmodule > threshold) {
                linear = set(fcurrent_base.point.x/alpha, fcurrent_base.point.y/alpha, fcurrent_base.point.z/alpha);
                angular = set(0, 0, 0);
            } else {
                linear = set(0, 0, 0);
                angular = set(0, 0, 0);
            }

            // linear is (0, 0, 0) if force < threshold or a position is saved
            move.linear = linear;
            move.angular = angular;

            publisher.publish(move);
        }

        ros::spinOnce();
        rate.sleep();
    }

    pos_publisher.publish(positions[0]);
    pos_publisher.publish(positions[1]);

    mySigintHandler(SIGINT);
    return 0;
}