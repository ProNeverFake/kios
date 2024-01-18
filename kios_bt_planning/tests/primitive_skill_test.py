
from kios_robot.robot_proprioceptor import *
from kios_robot.robot_actuator import *
from kios_robot.robot_interface import *

def test_6d_cartesian_move():
    '''
    under development
    '''
    # get current pose
    teach_location(MIOS, "kios_guidance")
    # modify in x
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")
    # modify in y
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)  
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")
    # modify in z
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")
    # modify in yaw
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")
    # modify in pitch
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")
    # modify in roll
    modify_taught_pose(MIOS, "kios_guidance", 0.417140386890134, 0.4047956531460382, 0.015098423457252558)
    # move to new pose
    move_to_cartesian_location(MIOS, "kios_guidance")






if __name__ == "__main__":
    print(test_connection())
    pass
    