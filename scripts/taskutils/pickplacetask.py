import collections
import copy
from dataclasses import dataclass
from typing import Union

import rospy
from geometry_msgs.msg import Point, Twist, Vector3, WrenchStamped
from moveit_commander.move_group import MoveGroupCommander
from rospy import Publisher
from taskutils.utils import UrControllersNames, switch_controller, zero_ft_sensor


class PickPlaceTask:
    """ Pick and Place Task
        -------------------
        This class is part of a pick and place task in which the robot must find the center position of a box to place an object.
        In this case we consider a box as an upper-elevated surface where the robot will drop the object. Thus the robot will try to find 
        all the border from outside the box, rather than from the inside. The robot will move up and down everytime, to prevent collision with box and
        to move to the next border.

        The starting position is considered to be the position in which the robot will start its execution. It will attempt to reach, in order:

        - the surface
        - the top border
        - the bottom border
        - the right border
        - the left border

        In this task the robot will move along y to reach top/bottom borders and along x to reach right and left borders. 
        The size of the box is not known,but some kind of spacing is necessary to move the robot outside the box. 
        Provide a spacing considering also that the robot will need to move freely without colliding with other object
        and without reaching an anomaly point.
    
    """

    force_buffer = collections.deque(maxlen=15)
    """A deque that will store the last 15 force readings."""

    contact_detection_paused = False
    """A semaphore variable to pause contact detection."""

    in_contact_threshold: float = 2
    """A float value to specify at which force the robot is considered to enter a contact with another object."""

    in_contact: bool = False
    """A boolean value indicating whether the robot is in contact state or not"""

    move_group: MoveGroupCommander
    """A reference to the move group"""

    moveit_speed: float
    """Speed of moveit movements"""

    twist_publisher: Publisher
    """A reference to the publish that will publish twist velocities"""

    movement_speed: float
    """Speed of movements when using twist controller"""

    height_offset: float
    """The height that will be added from top surface, but also that will be subtracted from top surface"""

    top_bottom_spacing: float
    """The safe spacing between top and bottom borders. The robot will move to (unknown position) + / - half spacing """

    left_right_spacing: float
    """The safe spacing between top and bottom borders. The robot will move to (unknown position) + / - half spacing """

    top_surface_position: Point
    top_border_position: Point
    bottom_border_position: Point
    right_border_position: Point
    left_border_position: Point

    @dataclass
    class MovementDirection:
        UP = 1
        DOWN = 2
        FORWARD = 3
        BACKWARD = 4
        RIGHT = 5
        LEFT = 6

    def __init__(self, move_group: MoveGroupCommander, twist_publisher: Publisher, movement_speed: float, moveit_speed: float, 
                 height_offset: float, top_bottom_spacing: float, left_right_spacing: float, 
                 in_contact_threshold: Union[float, None] = None) -> None:
        """ Constructor for PickPlaceTask.

            Args:
                move_group (MoveGroupCommander): The move group that the PickPlaceTask will control.

                twist_publisher (Publisher): The publisher that will publish the twist velocities.

                movement_speed (float): Speed of movements when using twist controller.

                movement_speed (float): Speed of moveit movements.

                height_offset (float): The height that will be added from top surface.

                top_bottom_spacing (float): The safe spacing between top and bottom borders. The robot will move to (unknown position) + / - half spacing.

                left_right_spacing (float): The safe spacing between top and bottom borders. The robot will move to (unknown position) + / - half spacing.

                in_contact_threshold (float): The threshold force at which the robot is considered to enter a contact with another object.


        """
        self.move_group = move_group
        self.twist_publisher = twist_publisher
        self.movement_speed = movement_speed
        self.moveit_speed = moveit_speed
        self.height_offset = height_offset
        self.top_bottom_spacing = top_bottom_spacing
        self.left_right_spacing = left_right_spacing

        if in_contact_threshold is not None:
            self.in_contact_threshold = in_contact_threshold


    def force_callback(self, message: WrenchStamped):
        """ A callback method which will be called from the Subscriber 
        
            Args:
                message (WrenchStamped): The message that will be received
        
        """
        self.force_buffer.append(message)
        # Check if buffer has less than 15 values
        if len(self.force_buffer) < 15:
            return
        
        # Check if contact detection is paused
        if self.contact_detection_paused:
            return

        # DEBUG
        #print(f"force detected: |{abs(force_detected.x)}|, |{abs(force_detected.y)}|")
        
        # Check if every of the last 15 packets is in contact
        for packet in self.force_buffer:
            if not self.check_contact_message(packet):
                self.in_contact = False
                return
        self.in_contact = True
        rospy.loginfo("Contact detected.")

    def check_contact_message(self, message: WrenchStamped) -> bool:
        """A utility method to check if a message represent a contact state."""
        force_detected = message.wrench.force
        if abs(force_detected.x) > self.in_contact_threshold or abs(force_detected.y)  > self.in_contact_threshold:
            return True
        return False
    
    def wait_until_contact(self, direction: int):
        """This method will wait for a contact, then it will move the robot out of contact"""
        buffer = collections.deque(maxlen=15)
        twist_movement: Twist = Twist()
        if direction not in range(1, 7):
            rospy.logwarn("No valid direction. Trying stopping all")
            twist_movement.linear = Vector3(0, 0, 0)
            self.twist_publisher.publish(twist_movement)
            return

        if direction == self.MovementDirection.FORWARD:
            twist_movement.linear = Vector3(0, -self.movement_speed, 0)
        elif direction == self.MovementDirection.BACKWARD:
            twist_movement.linear = Vector3(0, self.movement_speed, 0)
        elif direction == self.MovementDirection.RIGHT:
            twist_movement.linear = Vector3(-self.movement_speed, 0, 0)
        elif direction == self.MovementDirection.LEFT:
            twist_movement.linear = Vector3(self.movement_speed, 0, 0)
        elif direction == self.MovementDirection.UP:
            twist_movement.linear = Vector3(0, 0, -self.movement_speed)
        elif direction == self.MovementDirection.DOWN:
            twist_movement.linear = Vector3(0, 0, self.movement_speed)
        else:
            rospy.logwarn("Unable to detect movement direction!")
        
        # Wait for contact detection
        while True:
            if self.in_contact:
                break
        # Pause contact detection
        self.contact_detection_paused = True
        # DEBUG
        rospy.logwarn("CONTACT DETECTED! WAIT FOR EXIT CONDITION!")
        # Move back until contact is no longer detected
        self.twist_publisher.publish(twist_movement)
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
                if self.check_contact_message(packet):
                    self.in_contact = True
                    break
                self.in_contact = False

            # If all packets are not registering contact
            if not self.in_contact:
                break

            # continue checking
        
        # Stop moving
        twist_movement.linear = Vector3(0, 0, 0)
        self.twist_publisher.publish(twist_movement)
        # DEBUG
        rospy.logwarn("CONTACT NO LONGER DETECTED. RESUMING NORMAL OPERATION")
        # Reset force sensor
        zero_ft_sensor()
        # Resume contact detection
        self.contact_detection_paused = False


    def reach_point(self, point: Point):
        """Utility function to reach a point in space via moveit"""
        # Set target pose
        target_pose = self.move_group.get_current_pose().pose
        target_pose.position = point
        # Change controllers
        switch_controller(
            start_controllers=[UrControllersNames.scaled_pos_joint_traj_controller],
            stop_controllers=[UrControllersNames.twist_controller]
        )
        # Reach position 
        self.move_group.set_pose_target(target_pose)
        self.move_group.set_max_velocity_scaling_factor(self.moveit_speed)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        # Restore contollers
        switch_controller(
            start_controllers=[UrControllersNames.twist_controller],
            stop_controllers=[UrControllersNames.scaled_pos_joint_traj_controller]
        )


    def find_top_surface(self):
        """ 1 - Find top surface
            -
        """
        twist_movement = Twist()
        twist_movement.angular = Vector3(0, 0, 0)
        twist_movement.linear = Vector3(0, 0, -self.movement_speed) # Move down
        self.twist_publisher.publish(twist_movement)
        self.wait_until_contact(self.MovementDirection.DOWN)
        # Save top surface position
        self.top_surface_position = copy.deepcopy(self.move_group.get_current_pose().pose.position)
        rospy.loginfo(f"TOP POSITION:\n{self.top_surface_position}")


    def move_top_border(self):
        """ 2 - Move to top border
            -
        """
        # Move up, forward and down
        current_position = self.move_group.get_current_pose().pose.position
        current_position.z += self.height_offset
        self.reach_point(current_position) # Move UP offset
        current_position.y += self.top_bottom_spacing / 2
        self.reach_point(current_position) # Move forward
        current_position.z -= self.height_offset * 2
        self.reach_point(current_position) # Move down

    
    def find_top_border(self):
        """ 3 - Find top border
            -
        """
        # Move backward till contact
        twist_movement = Twist()
        twist_movement.linear = Vector3(0, -self.movement_speed, 0)
        self.twist_publisher.publish(twist_movement)
        self.wait_until_contact(self.MovementDirection.BACKWARD)
        # Save position
        self.top_border_position = copy.deepcopy(self.move_group.get_current_pose().pose.position)
        rospy.loginfo(f"FORDWARD BORDER POSITION:\n{self.top_border_position}")

    def move_bottom_border(self):
        """ 4 - Move bottom border
            -
        """
        # Move up, backward, down
        current_position = self.move_group.get_current_pose().pose.position
        current_position.z += self.height_offset * 2
        self.reach_point(current_position) # Move UP
        current_position.y -= self.top_bottom_spacing
        self.reach_point(current_position) # Move BACK
        current_position.z -= self.height_offset * 2
        self.reach_point(current_position)

    
    def find_bottom_border(self):
        """ 5 - Find top border
            -
        """
        twist_movement = Twist()
        twist_movement.linear = Vector3(0, self.movement_speed, 0)
        self.twist_publisher.publish(twist_movement)
        self.wait_until_contact(self.MovementDirection.FORWARD)
        # Save position
        self.bottom_border_position = copy.deepcopy(self.move_group.get_current_pose().pose.position)
        rospy.loginfo(f"BACKWARD BORDER POSITION:\n{self.bottom_border_position}")


    def move_top_bottom_center(self):
        """ 6 - Move top/bottom center
            -
        """
        current_position = self.move_group.get_current_pose().pose.position
        current_position.z += self.height_offset * 2
        self.reach_point(current_position) # Move UP
        current_position.y = (self.top_border_position.y + self.bottom_border_position.y)/2
        self.reach_point(current_position) # Move center
        rospy.loginfo(f"CENTER FB POSITION:\n{current_position}")
    

    def move_right_border(self):
        """ 7 - Move right border
            -
        """
        # Move right, down
        current_position = self.move_group.get_current_pose().pose.position
        current_position.x += self.left_right_spacing / 2
        self.reach_point(current_position) # Move right
        current_position.z -= self.height_offset * 2
        self.reach_point(current_position) # Move down

    
    def find_right_border(self):
        """ 8 - Find right border
            -
        """
        # Move left till contact
        twist_movement = Twist()
        twist_movement.linear = Vector3(-self.movement_speed, 0, 0)
        self.twist_publisher.publish(twist_movement)
        self.wait_until_contact(self.MovementDirection.LEFT)
        # Save position
        self.right_border_position = copy.deepcopy(self.move_group.get_current_pose().pose.position)
        rospy.loginfo(f"RIGHT BORDER POSITION:\n{self.right_border_position}")

    
    def move_left_border(self):
        """ 9 - Move left border
            -
        """
        current_position = self.move_group.get_current_pose().pose.position
        current_position.z += self.height_offset * 2
        self.reach_point(current_position) # Move UP
        current_position.x -= self.left_right_spacing
        self.reach_point(current_position) # Move left
        current_position.z -= self.height_offset * 2
        self.reach_point(current_position) # Move down

    
    def find_left_border(self):
        """ 10 - Move left border
            -
        """
        # Move right till contact
        twist_movement = Twist()
        twist_movement.linear = Vector3(self.movement_speed, 0, 0)
        self.twist_publisher.publish(twist_movement)
        self.wait_until_contact(self.MovementDirection.RIGHT)
        # Save position
        self.left_border_position = copy.deepcopy(self.move_group.get_current_pose().pose.position)
        rospy.loginfo(f"LEFT BORDER POSITION:\n{self.left_border_position}")

    
    def move_center(self):
        """ 11 - Move to center
            -
        """
        # Move up
        current_position = self.move_group.get_current_pose().pose.position
        current_position.z += self.height_offset * 2
        self.reach_point(current_position) # Move UP

        # Calculate center x, y
        current_position.x = (self.left_border_position.x + self.right_border_position.x)/2.0
        current_position.y = (self.top_border_position.y + self.bottom_border_position.x)/2
        self.reach_point(current_position)
    
    
