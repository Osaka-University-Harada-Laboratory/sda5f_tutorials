#!/usr/bin/env python

import math
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander


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
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def pick_and_place():
    """Executes pick-and-place motions."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()
    rarm = MoveGroupCommander("arm_right")

    # get current status
    rarm_initial_pose = rarm.get_current_pose().pose
    rarm_initial_joint_values = rarm.get_current_joint_values()

    # set maximum velocity and acceleration
    rarm.set_max_velocity_scaling_factor(0.5)
    rarm.set_max_acceleration_scaling_factor(0.5)

    # right arm pose above object
    p1_jvs = [3.814509,
              -0.300463,
              -2.478898,
              0.001337,
              -1.326666,
              -1.775833,
              1.812151]
    # right arm pose to grasp
    p2_jvs = [4.072096,
              -0.224721,
              -2.261118,
              -0.141035,
              -1.462992,
              -1.302057,
              1.862357]
    # right arm pose above target place to release
    p3_jvs = [1.688017,
              -1.102127,
              -0.394552,
              -0.109065,
              -1.419403,
              -1.898947,
              0.621314]
    # right arm pose to release
    p4_jvs = [1.412113,
              -1.184674,
              0.088591,
              -0.244860,
              -1.715401,
              -1.581672,
              0.634262]

    # execute pick-and-place motions using right arm
    rospy.loginfo("Start right arm pick-and-place motions.")
    cnt = 0
    while cnt < 1:
        rospy.loginfo("Moving to above object...")
        go_with_joint_values(rarm, p1_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to grasp object...")
        go_with_joint_values(rarm, p2_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to above object...")
        go_with_joint_values(rarm, p1_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to above target place...")
        go_with_joint_values(rarm, p3_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to release object...")
        go_with_joint_values(rarm, p4_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to above target place...")
        go_with_joint_values(rarm, p3_jvs)
        show_joint_values(rarm)
        cnt += 1

    # initialize joints
    rospy.loginfo("Initializing right arm joint values...")
    rarm.set_joint_value_target(rarm_initial_joint_values)
    rarm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
