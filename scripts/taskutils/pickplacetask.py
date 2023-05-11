

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
