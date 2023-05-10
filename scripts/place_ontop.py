import collections
import os
from dataclasses import dataclass
import sys
from typing import List
from moveit_commander.roscpp_initializer import roscpp_initialize
import rospy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (ListControllers,
                                         ListControllersResponse,
                                         LoadController, LoadControllerRequest,
                                         LoadControllerResponse,
                                         SwitchController,
                                         SwitchControllerRequest,
                                         SwitchControllerResponse,
                                         UnloadController,
                                         UnloadControllerRequest,
                                         UnloadControllerResponse)
from geometry_msgs.msg import Vector3, WrenchStamped
from robotiq_ft_sensor.srv import (sensor_accessor, sensor_accessorRequest,
                                   sensor_accessorResponse)
from rospy import Publisher

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

def unload_controller(controller_name: str) -> UnloadControllerResponse:
    load_controller_service = rospy.ServiceProxy("/controller_manager/unload_controller", UnloadController)
    request = UnloadControllerRequest(controller_name)
    response: UnloadControllerResponse = load_controller_service.call(request)

def switch_controller(start_controllers: List[str], stop_controllers: List[str], strictness: int = 1):
    switch_controller_service = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    request = SwitchControllerRequest(start_controllers, stop_controllers, strictness, False, 0)
    response: SwitchControllerResponse = switch_controller_service.call(request)
    return response

def zero_ft_sensor():
    try:
        zero_sensor_service = rospy.ServiceProxy("/robotiq_ft_sensor_acc", sensor_accessor)
        request = sensor_accessorRequest(sensor_accessorRequest.COMMAND_SET_ZERO, "")
        response: sensor_accessorResponse = zero_sensor_service.call(request)
        rospy.sleep(0.5)
        print(f"Sensor zeored: {response}")
        return response
    except rospy.service.ServiceException:
        rospy.logwarn("Unable to connect to /robotiq_ft_sensor_acc and zero f/t sensor")

def shutdown():
    global movement
    global publisher
    rospy.logwarn("Initiating inner_border_finder shutdown")
    movement.linear = Vector3(0, 0, 0)
    publisher.publish(movement)
    switch_controller(
        start_controllers=[UrControllersNames.scaled_pos_joint_traj_controller],
        stop_controllers=[UrControllersNames.twist_controller]
    )
    os._exit(1) #Kill parent process

def detect_contact(message: WrenchStamped):
    global in_contact
    global pause_contact_detection
    global in_contact_threshold
    global message_buffer
    message_buffer.append(message)
    if len(message_buffer) < 15:
        return
    # Check if contact detection is paused
    if pause_contact_detection:
        return

    # DEBUG
    #print(f"force detected: |{abs(force_detected.x)}|, |{abs(force_detected.y)}|")
    
    # Check if everyone of the last 15 packets are in contact
    for packet in message_buffer:
        if not check_contact_message(packet):
            in_contact = False
            return
    in_contact = True
    rospy.loginfo("Contact detected.")

def check_contact_message(message: WrenchStamped):
    force_detected = message.wrench.force
    if abs(force_detected.x) > in_contact_threshold or abs(force_detected.y)  > in_contact_threshold:
        return True
    return False

def wait_until_contact(direction: int):
    global publisher
    global movement
    global in_contact
    global pause_contact_detection
    global out_contact_threshold
    global speed
    buffer = collections.deque(maxlen=15)
    if direction not in range(1, 5):
        print("Error. No valid direction. Trying stopping all")
        movement.linear = Vector3(0, 0, 0)
        publisher.publish(movement)
        return

    if direction == 1:
        movement.linear = Vector3(0, -speed, 0)
    elif direction == 2:
        movement.linear = Vector3(0, speed, 0)
    elif direction == 3:
        movement.linear = Vector3(-speed, 0, 0)
    else:
        movement.linear = Vector3(speed, 0, 0)
    
    # Wait for contact detection
    while True:
        if in_contact:
            break
    # Pause contact detection
    pause_contact_detection = True
    # DEBUG
    rospy.logwarn("CONTACT DETECTED! WAIT FOR EXIT CONDITION!")
    # Move back until contact is no longer detected
    publisher.publish(movement)
    # DEBUG
    rospy.loginfo("Starting moving back.")

    # Wait for contact exit
    while True:
        message: WrenchStamped = rospy.wait_for_message("robotiq_ft_wrench", WrenchStamped)
        buffer.append(message)
        if len(buffer) < 15:
            continue # Contiunue appending messages

        # If 15 messages collected
        for packet in buffer:
            if check_contact_message(packet):
                in_contact = True
                break
            in_contact = False

        # If all packets are not registering contact
        if not in_contact:
            break

        # continue checking
    
    # Stop moving
    movement.linear = Vector3(0, 0, 0)
    publisher.publish(movement)
    # DEBUG
    rospy.logwarn("CONTACT NO LONGER DETECTED. RESUMING NORMAL OPERATION")
    # Reset force sensor
    zero_ft_sensor()
    # Resume contact detection
    pause_contact_detection = False

# Global variables
speed: float
movement: Vector3
publisher: Publisher
in_contact_threshold: float = 2
out_contact_threshold: float = 0.7
# Semaphores for contact detection
in_contact: bool = False
pause_contact_detection: bool = False
# Buffer
message_buffer = collections.deque(maxlen=15)

def main():

    # Initialize node and shutdown function
    rospy.init_node("place_ontop")
    rospy.on_shutdown(shutdown)
    #Initialize MoveIt
    roscpp_initialize(sys.argv)

    pass

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        exit()