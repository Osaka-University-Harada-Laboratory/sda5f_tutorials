#!/usr/bin/env python3

import math
import rospy
from moveit_msgs.msg import DisplayTrajectory
from moveit_commander import (MoveGroupCommander,
                              RobotCommander,
                              PlanningSceneInterface)

from industrial_msgs.msg import RobotStatus
from sda5f_motion_plan.srv import SetCommand

sda5f_e_stopped = 0
sda5f_motion_possible = 0
sda5f_in_motion = 0


def save_sda5f_status(status):
    """Saves the current sda5f status."""
    global sda5f_e_stopped, sda5f_motion_possible, sda5f_in_motion
    sda5f_e_stopped = status.e_stopped.val
    sda5f_motion_possible = status.motion_possible.val
    sda5f_in_motion = status.in_motion.val
    # rospy.loginfo("sda5f: "+str(sda5f_in_motion))


def show_joint_values(mgc):
    """Shows joint values with rospy log info."""
    rjs = mgc.get_current_joint_values()
    rospy.loginfo("Current joint angles.")
    rospy.loginfo("j1: %f", rjs[0])
    rospy.loginfo("j2: %f", rjs[1])
    rospy.loginfo("j3: %f", rjs[2])
    rospy.loginfo("j4: %f", rjs[3])
    rospy.loginfo("j5: %f", rjs[4])
    rospy.loginfo("j6: %f", rjs[5])
    rospy.loginfo("j7: %f", rjs[6])


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go(wait=True)
    rospy.sleep(rospy.Duration.from_sec(0.5))


def demo_wavearms():
    """Executes sda5f whole body motions."""
    global sda5f_e_stopped, sda5f_motion_possible, sda5f_in_motion
    rospy.init_node(
        "moveit_command_sender",
        disable_signals=True,
        log_level=rospy.DEBUG)
    rospy.Subscriber("robot_status", RobotStatus, save_sda5f_status)
    rospy.sleep(rospy.Duration.from_sec(1))

    if sda5f_e_stopped == 1:
        rospy.loginfo("Error on sda5f: Emergency is actvated.\n")
        rospy.signal_shutdown("Finished.")
        rospy.sleep(rospy.Duration.from_sec(1))
        return -1
    if sda5f_motion_possible == 0:
        rospy.loginfo(
            "Error on sda5f: Motion is not possible.",
            "Motor is not enabled.\n")
        rospy.signal_shutdown("Finished.")
        rospy.sleep(rospy.Duration.from_sec(1))
        return -1
    # left (72) or right (104) finger's safety swith is activated

    torso = MoveGroupCommander("torso")
    larm = MoveGroupCommander("arm_left")
    rarm = MoveGroupCommander("arm_right")
    arms = MoveGroupCommander("arms")
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20)

    arms.set_max_velocity_scaling_factor(1.0)
    arms.set_max_acceleration_scaling_factor(1.0)
    torso.set_max_velocity_scaling_factor(0.8)
    torso.set_max_acceleration_scaling_factor(0.8)

    # Initializing whole body joint values
    initial_arms_jvs = [
        math.radians(170.0),
        math.radians(-90.0),
        math.radians(-90.0),
        math.radians(-10.0),
        math.radians(0.0),
        math.radians(-80.0),
        math.radians(0.0),
        math.radians(170.0),
        math.radians(-90.0),
        math.radians(-90.0),
        math.radians(-10.0),
        math.radians(0.0),
        math.radians(-80.0),
        math.radians(0.0)]
    wave_arms_jvs = [
        math.radians(150.0),
        math.radians(-90.0),
        math.radians(-90.0),
        math.radians(50.0),
        math.radians(0.0),
        math.radians(20.0),
        math.radians(0.0),
        math.radians(150.0),
        math.radians(-90.0),
        math.radians(-90.0),
        math.radians(50.0),
        math.radians(0.0),
        math.radians(20.0),
        math.radians(0.0)]

    initial_torso_jvs = [0., 0.]
    go_with_joint_values(torso, initial_torso_jvs)
    while not rospy.is_shutdown():
        rospy.loginfo("Initializing whole body joints...")
        go_with_joint_values(arms, initial_arms_jvs)

        larm_initial_pose = larm.get_current_pose().pose
        larm_initial_joint_values = larm.get_current_joint_values()
        rospy.loginfo("larm initial configuration")
        rospy.loginfo(larm_initial_pose)
        rospy.loginfo(larm_initial_joint_values)
        rarm_initial_pose = rarm.get_current_pose().pose
        rarm_initial_joint_values = rarm.get_current_joint_values()
        rospy.loginfo("rarm initial configuration")
        rospy.loginfo(rarm_initial_pose)
        rospy.loginfo(rarm_initial_joint_values)

        go_with_joint_values(arms, wave_arms_jvs)

    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        demo_wavearms()
    except rospy.ROSInterruptException:
        pass
