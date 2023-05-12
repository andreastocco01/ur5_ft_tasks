import os
import sys

import rospy
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from rospy import Publisher
from taskutils.utils import UrControllersNames, load_controller, switch_controller, zero_ft_sensor
from taskutils.pickplacetask import PickPlaceTask
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.roscpp_initializer import roscpp_initialize



twist_publisher: Publisher
"""The Publisher that will publish the Twist messages"""

def shutdown() -> None:
    """ Define what to do when node is shutting down. This will attempt to stop velocity movements and restore default controller

        Args:
            publisher (Publisher): where the velocity commands are being sent, to publish a velocity of Vector3(0, 0, 0)
    """
    global twist_publisher
    stop_movement: Twist = Twist()
    rospy.logwarn("Initiating shutdown")
    stop_movement.linear = Vector3(0, 0, 0)
    twist_publisher.publish(stop_movement)
    switch_controller(
        start_controllers=[UrControllersNames.scaled_pos_joint_traj_controller],
        stop_controllers=[UrControllersNames.twist_controller]
    )
    os._exit(1) #Kill parent process

def main():

    # Initialize node and shutdown function
    rospy.init_node("pick_place")
    rospy.on_shutdown(shutdown)
    #Initialize MoveIt
    roscpp_initialize(sys.argv)

    # Create a new moveit movegroup commander for arm group
    group_name = "ur5_arm"
    move_group = MoveGroupCommander(group_name)

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
    twist_publisher = rospy.Publisher("/twist_controller/command", Twist, queue_size=1)
    # Wait until a node is connected
    while not twist_publisher.get_num_connections() > 0:
        rospy.logwarn("No active connections at the moment. Waiting for connections . . . ")
        rospy.sleep(0.5)

    # Create a PickPlaceTask
    movement_speed = 0.01
    height_offset = 0.04
    top_bottom_spacing = -0.25 #TODO: fix sign
    left_right_spacing = -0.25 #TODO: fix sign
    moveit_speed = 0.5
    task = PickPlaceTask(move_group, twist_publisher, movement_speed, moveit_speed, height_offset, top_bottom_spacing, left_right_spacing)

    # Subscribe to robotiq sensor topic
    subscriber = rospy.Subscriber("robotiq_ft_wrench", WrenchStamped, task.force_callback)

    # Zeroing the sensor
    zero_ft_sensor()

    # Start moving robot

    # Move down till contact
    task.find_top_surface()

    # Move top border
    task.move_top_border()

    # Move backward till contact
    task.find_top_border()

    # Move bottom border
    task.move_bottom_border()

    # Move top/bottom center
    task.move_top_bottom_center()

    # Find right border
    task.find_right_border()

    # Move left border
    task.move_left_border()

    # Find left border
    task.find_left_border()

    # Move center
    task.move_center()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        exit()