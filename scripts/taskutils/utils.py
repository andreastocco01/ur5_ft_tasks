from dataclasses import dataclass
from typing import List, Union

import rospy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (ListControllers,
                                         ListControllersResponse,
                                         LoadController, LoadControllerRequest, SwitchController, SwitchControllerRequest, SwitchControllerResponse,
                                         LoadControllerResponse,
                                         UnloadController,
                                         UnloadControllerRequest,
                                         UnloadControllerResponse)
from robotiq_ft_sensor.srv import sensor_accessor, sensor_accessorRequest, sensor_accessorResponse


@dataclass
class UrControllersNames:
    """Dataclass for storing names of UR controllers

    Use this class and its constants to reference one or
    more controllers by their names. Extended documentation is
    available at https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/controllers.md
    
    """

    joint_state_controller: str = "joint_state_controller"
    """ Read-only controller
        ---------------------
        Publishes all joints' positions, velocities, and motor currents as sensor_msgs/JointState on the /joint_states topic.
    """

    robot_status_controller: str = "robot_status_controller"
    """ Read-only controller
        ---------------------
        Controller that publishes robot/controller state as industrial_msgs/RobotStatus messages on the /robot_status topic.
    """

    force_torque_sensor_controller: str = "force_torque_sensor_controller"
    """ Read-only controller
        ---------------------
        This controller publishes the wrench measured from the robot at its TCP as geometry_msgs/WrenchStamped on the topic /wrench. 
        The wrench can be tared to 0 using the zero_ftsensor service.
    """

    speed_scaling_state_controller: str = "speed_scaling_state_controller"
    """ Read-only controller
        ---------------------
        Reports the current value of the speed multiplier used from the robot.
    """


    # ----------------------------------------------------
    scaled_pos_joint_traj_controller: str = "scaled_pos_joint_traj_controller"
    """ Commanding controller
        ----------------------
        For controlling the robot it will send joint position commands.

    """

    joint_group_vel_controller: str = "joint_group_vel_controller"
    """ Commanding controller
        ----------------------
        Directly command joint velocities though a topic interface.

    """

    twist_controller: str = "twist_controller"
    """ Commanding controller
        ----------------------
        This controller allows directly sending speeds via a topic interface to the robot. However, this controller expects a Cartesian TCP twist (linear and angular velocity).

    """

    # ----------------------------------------------------
    pos_joint_traj_controller: str = "pos_joint_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    vel_joint_traj_controller: str = "vel_joint_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    scaled_vel_joint_traj_controller: str = "scaled_vel_joint_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    forward_joint_traj_controller: str = "forward_joint_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    forward_cartesian_traj_controller: str = "forward_cartesian_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    pose_based_cartesian_traj_controller: str = "pose_based_cartesian_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    joint_based_cartesian_traj_controller: str = "joint_based_cartesian_traj_controller"
    """ Commanding controller
        ----------------------
        

    """

    # ----------------------------------------------------
    all_commanding_controllers: List[str] = [
        scaled_pos_joint_traj_controller,
        joint_group_vel_controller,
        twist_controller,
        pos_joint_traj_controller,
        vel_joint_traj_controller,
        scaled_vel_joint_traj_controller,
        forward_joint_traj_controller,
        forward_cartesian_traj_controller,
        pose_based_cartesian_traj_controller,
        joint_based_cartesian_traj_controller
    ]
    """This variable is a list containing all controllers' names in this dataclass"""


# Utility functions

def list_controllers() -> ListControllersResponse:
    """ Prints a list of controllers. Return a ListControllerReponse.

    Returns:
        ListControllersResponse: The response from calling controller_manager/list_controllers

    """
    list_controllers_service = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    response: ListControllersResponse = list_controllers_service.call()
    elem: ControllerState
    for elem in response.controller: # type: ignore
        print(f"{elem.name}:{elem.state}")
    return response


def load_controller(controller_name: str) -> LoadControllerResponse:
    """ Load a controller in controller_manager.

    Args:
        controller_name (str): The name of the controller to load

    Returns:
        LoadControllerResponse: The response from calling controller_manager/load_controller

    """
    load_controller_service = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    request = LoadControllerRequest(controller_name)
    response: LoadControllerResponse =load_controller_service.call(request)
    return response


def unload_controller(controller_name: str) -> UnloadControllerResponse:
    """ Unload a controller in controller_manager.

    Args:
        controller_name (str): The name of the controller to unload

    Returns:
        UnloadControllerResponse: The response from calling controller_manager/unload_controller

    """
    load_controller_service = rospy.ServiceProxy("/controller_manager/unload_controller", UnloadController)
    request = UnloadControllerRequest(controller_name)
    response: UnloadControllerResponse = load_controller_service.call(request)
    return response


def switch_controller(start_controllers: List[str], stop_controllers: List[str], strictness: int = 1) -> SwitchControllerResponse:
    """ Switch two or more controllers in controller_manager.

    Args:
        start_controllers (list[str]): The names of the controllers to start

        stop_controllers (list[str]): The names of the controllers to stop

        strictness (int): An integer indicating level of strictenss to start/stop controllers. 
        Can be either 1 (BEST_EFFORT) or 2 (STRICT). STRICT means that switching will fail in case anything goes wrong.

    Returns:
        SwitchControllerResponse: The response from calling controller_manager/switch_controller

    """
    switch_controller_service = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    request = SwitchControllerRequest(start_controllers, stop_controllers, strictness, False, 0)
    response: SwitchControllerResponse = switch_controller_service.call(request)
    return response


def zero_ft_sensor() -> Union[sensor_accessorResponse, None]:
    """ This function will call service /robotiq_ft_sensor_acc to request zeroing of force/torque sensor"""
    try:
        zero_sensor_service = rospy.ServiceProxy("/robotiq_ft_sensor_acc", sensor_accessor)
        request = sensor_accessorRequest(sensor_accessorRequest.COMMAND_SET_ZERO, "")
        response: sensor_accessorResponse = zero_sensor_service.call(request)
        rospy.sleep(0.5)
        print(f"Sensor zeored: {response}")
        return response
    except rospy.service.ServiceException:
        rospy.logwarn("Unable to connect to /robotiq_ft_sensor_acc and zero f/t sensor")