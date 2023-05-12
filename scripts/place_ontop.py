import collections
import os
import sys
from dataclasses import dataclass
from typing import List
import copy

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
from geometry_msgs.msg import Vector3, WrenchStamped, Twist, Pose, Point
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.roscpp_initializer import roscpp_initialize
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

@dataclass
class Movement:
    UP = 1
    DOWN = 2
    FORWARD = 3
    BACKWARD = 4
    RIGHT = 5
    LEFT = 6

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
    global speed
    buffer = collections.deque(maxlen=15)
    if direction not in range(1, 7):
        rospy.logwarn("No valid direction. Trying stopping all")
        movement.linear = Vector3(0, 0, 0)
        publisher.publish(movement)
        return

    if direction == Movement.FORWARD:
        movement.linear = Vector3(0, -speed, 0)
    elif direction == Movement.BACKWARD:
        movement.linear = Vector3(0, speed, 0)
    elif direction == Movement.RIGHT:
        movement.linear = Vector3(-speed, 0, 0)
    elif direction == Movement.LEFT:
        movement.linear = Vector3(speed, 0, 0)
    elif direction == Movement.UP:
        movement.linear = Vector3(0, 0, -speed)
    elif direction == Movement.DOWN:
        movement.linear = Vector3(0, 0, speed)
    else:
        rospy.logwarn("Unable to detect movement direction!")
    
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
        message: WrenchStamped = rospy.wait_for_message("robotiq_ft_wrench", WrenchStamped) # type: ignore
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

def reach_point(point: Point):
    global moveit_speed
    # Set target pose
    target_pose = move_group.get_current_pose().pose
    target_pose.position = point
    # Change controllers
    switch_controller(
        start_controllers=[UrControllersNames.scaled_pos_joint_traj_controller],
        stop_controllers=[UrControllersNames.twist_controller]
    )
    # Reach position 
    move_group.set_pose_target(target_pose)
    move_group.set_max_velocity_scaling_factor(moveit_speed)
    move_group.go(wait=True)
    move_group.stop()
    #move_group.clear_pose_targets()
    # Restore contollers
    switch_controller(
        start_controllers=[UrControllersNames.twist_controller],
        stop_controllers=[UrControllersNames.scaled_pos_joint_traj_controller]
    )
    rospy.logerr(f"Volevo raggingere {target_pose.position}\nHo raggiunto {move_group.get_current_pose().pose.position}")

# TODO: Fix signs of offsets - apparently moveit and twist controller have different reference frames
# Global variables
speed: float = 0.01
v_offset: float = 0.04 # Altezza
h_offset: float = -0.15 #TODO: fix sign
length_offset: float = -0.25 #TODO: fix sign
movement: Twist
publisher: Publisher
in_contact_threshold: float = 2
#out_contact_threshold: float = 0.7
move_group: MoveGroupCommander
moveit_speed = 0.5
# Semaphores for contact detection
in_contact: bool = False
pause_contact_detection: bool = False
# Buffer
message_buffer = collections.deque(maxlen=15)

def main():

    
    global speed
    global v_offset
    global h_offset
    global length_offset
    global movement
    global publisher
    global move_group

    # Initialize node and shutdown function
    rospy.init_node("place_ontop")
    rospy.on_shutdown(shutdown)
    #Initialize MoveIt
    roscpp_initialize(sys.argv)

    # Create a new moveit movegroup commander for arm group
    group_name = "ur5_arm"
    move_group = MoveGroupCommander(group_name)
    name = move_group.get_planning_frame()
    print(name)
    
    
    # Subscribe to robotiq sensor topic
    subscriber = rospy.Subscriber("robotiq_ft_wrench", WrenchStamped, detect_contact)

    # Load the correct controller
    load_controller(UrControllersNames.twist_controller)
    switch_controller(
        start_controllers=[UrControllersNames.twist_controller],
        stop_controllers=[UrControllersNames.scaled_pos_joint_traj_controller]
    )

    # Check that a /twist_controller/command topic exists
    while not rospy.core.is_topic("/twist_controller/command"):
        pass
    # Publish to twist controller topic and set rospin rate
    publisher = rospy.Publisher("/twist_controller/command", Twist, queue_size=1)
    # Wait until a node is connected
    while not publisher.get_num_connections() > 0:
        rospy.logwarn("No active connections at the moment. Waiting for connections . . . ")
        rospy.sleep(0.5)

    zero_ft_sensor()

    # Start moving robot

    # Move down till contact
    movement = Twist()
    movement.angular = Vector3(0, 0, 0)
    movement.linear = Vector3(0, 0, -speed) # Move down
    publisher.publish(movement)
    wait_until_contact(Movement.DOWN)
    # Save position
    top_position = copy.deepcopy(move_group.get_current_pose().pose.position)
    rospy.loginfo(f"TOP POSITION:\n{top_position}")

    # Move up, forward and down
    current_position = top_position
    current_position.z += v_offset
    reach_point(current_position) # Move UP offset
    current_position.y += length_offset / 2
    reach_point(current_position) # Move forward
    current_position.z -= v_offset * 2
    reach_point(current_position) # Move down

    # Move backward till contact
    movement.linear = Vector3(0, -speed, 0)
    publisher.publish(movement)
    wait_until_contact(Movement.BACKWARD)
    # Save position
    forward_border_position = copy.deepcopy(move_group.get_current_pose().pose.position)
    rospy.loginfo(f"FORDWARD BORDER POSITION:\n{forward_border_position}")

    # Move up, backward, down
    current_position = copy.deepcopy(forward_border_position)
    current_position.z += v_offset * 2
    reach_point(current_position) # Move UP
    current_position.y -= length_offset
    reach_point(current_position) # Move BACK
    current_position.z -= v_offset * 2
    reach_point(current_position)

    # Move forward till contact
    movement.linear = Vector3(0, speed, 0)
    publisher.publish(movement)
    wait_until_contact(Movement.FORWARD)
    # Save position
    backward_border_position = copy.deepcopy(move_group.get_current_pose().pose.position)
    rospy.loginfo(f"BACKWARD BORDER POSITION:\n{backward_border_position}")

    # Move up, center
    current_position = copy.deepcopy(backward_border_position)
    current_position.z += v_offset * 2
    reach_point(current_position) # Move UP
    current_position.y = (backward_border_position.y + forward_border_position.y)/2.0
    y_center = copy.deepcopy(current_position.y)
    rospy.loginfo(f"[{backward_border_position.y}], [{forward_border_position.y}] => [{current_position.y}]")
    reach_point(current_position) # Move center
    rospy.loginfo(f"CENTER FB POSITION:\n{current_position}")
    

    # Move right, down
    current_position.x += h_offset
    reach_point(current_position) # Move right
    current_position.z -= v_offset * 2
    reach_point(current_position) # Move down

    # Move left till contact
    movement.linear = Vector3(-speed, 0, 0)
    publisher.publish(movement)
    wait_until_contact(Movement.LEFT)
    # Save position
    right_border_position = copy.deepcopy(move_group.get_current_pose().pose.position)
    rospy.loginfo(f"RIGHT BORDER POSITION:\n{right_border_position}")

    # Move up, left, down
    current_position = copy.copy(right_border_position)
    current_position.z += v_offset * 2
    reach_point(current_position) # Move UP
    current_position.x -= h_offset * 2
    reach_point(current_position) # Move left
    current_position.z -= v_offset * 2
    reach_point(current_position) # Move down

    # Move right till contact
    movement.linear = Vector3(speed, 0, 0)
    publisher.publish(movement)
    wait_until_contact(Movement.RIGHT)
    # Save position
    left_border_position = copy.deepcopy(move_group.get_current_pose().pose.position)
    rospy.loginfo(f"LEFT BORDER POSITION:\n{forward_border_position}")

    # Move up
    current_position = copy.copy(left_border_position)
    current_position.z += v_offset * 2
    reach_point(current_position) # Move UP

    # Calculate center x, y
    current_position.x = (left_border_position.x + right_border_position.x)/2.0
    current_position.y = y_center
    rospy.loginfo(f"Desidered position X: {left_border_position.x} + {right_border_position.x} / 2 =\n= {current_position.x}")
    reach_point(current_position)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        exit()