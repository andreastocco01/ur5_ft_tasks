from dataclasses import dataclass

import rospy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (ListControllers,
                                         ListControllersResponse,
                                         LoadController, LoadControllerRequest,
                                         LoadControllerResponse)

'''
UR Controllers list (from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md)

Read only controllers:

    joint_state_controller
        Publishes all joints' positions, velocities, and motor currents as sensor_msgs/JointState on the /joint_states topic

    robot_status_controller
        Controller that publishes robot/controller state as industrial_msgs/RobotStatus messages on the /robot_status topic.
    
    force_torque_sensor_controller
        This controller publishes the wrench measured from the robot at its TCP as geometry_msgs/WrenchStamped on the topic /wrench. The wrench can be tared to 0 using the zero_ftsensor service.
    
    speed_scaling_state_controller
        Reports the current value of the speed multiplier used from the robot.

Commanding controllers (only one active at one time)
    --------- (recommended) ---------
    scaled_pos_joint_traj_controller
        For controlling the robot it will send joint position commands.
    joint_group_vel_controller
        Directly command joint velocities though a topic interface.
    twist_controller
        This controller allows directly sending speeds via a topic interface to the robot. However, this controller expects a Cartesian TCP twist (linear and angular velocity).
    config:
        type: "ros_controllers_cartesian/TwistController"
        frame_id: "tool0_controller"
        publish_rate: *loop_hz
        joints: *robot_joints
        


    --------- (others) ---------
    pos_joint_traj_controller
    vel_joint_traj_controller
    scaled_vel_joint_traj_controller
    forward_joint_traj_controller
    forward_cartesian_traj_controller
    pose_based_cartesian_traj_controller
    joint_based_cartesian_traj_controller
'''

@dataclass
class UrControllersNames:
    joint_state_controller = "joint_state_controller"
    robot_status_controller = "robot_status_controller"
    force_torque_sensor_controller = "force_torque_sensor_controller"
    speed_scaling_state_controller = "speed_scaling_state_controller"
    # ----------------------------------------------------
    scaled_pos_joint_traj_controller = "scaled_pos_joint_traj_controller"
    joint_group_vel_controller = "joint_group_vel_controller"
    twist_controller = "twist_controller"
    # ----------------------------------------------------
    pos_joint_traj_controller = "pos_joint_traj_controller"
    vel_joint_traj_controller = "vel_joint_traj_controller"
    scaled_vel_joint_traj_controller = "scaled_vel_joint_traj_controller"
    forward_joint_traj_controller = "forward_joint_traj_controller"
    forward_cartesian_traj_controller = "forward_cartesian_traj_controller"
    pose_based_cartesian_traj_controller = "pose_based_cartesian_traj_controller"
    joint_based_cartesian_traj_controller = "joint_based_cartesian_traj_controller"
    # ----------------------------------------------------
    all_commanding_controllers = [scaled_pos_joint_traj_controller,
                                  joint_group_vel_controller,
                                  twist_controller,
                                  pos_joint_traj_controller,
                                  vel_joint_traj_controller,
                                  scaled_vel_joint_traj_controller,
                                  forward_joint_traj_controller,
                                  forward_cartesian_traj_controller,
                                  pose_based_cartesian_traj_controller,
                                  joint_based_cartesian_traj_controller]
    
# Utility functions
def list_controllers() -> ListControllersResponse:
    list_controllers_service = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    response: ListControllersResponse = list_controllers_service.call()
    elem: ControllerState
    for elem in response.controller:
        print(f"{elem.name}:{elem.state}")
    return response

def load_controller(controller_name: str) -> LoadControllerResponse:
    load_controller_service = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    request = LoadControllerRequest(controller_name)
    response: LoadControllerResponse =load_controller_service.call(request)
    return response