#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_commander import RobotCommander, MoveGroupCommander
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def euler_to_quaternion(euler):
    """Converts euler angles to quaternion.

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def quaternion_to_euler(quaternion):
    """Converts quaternion to euler angles.

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])


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


def pick_and_toss():
    """Executes pick-and-toss motions."""
    rospy.init_node("moveit_command_sender")
    robot = RobotCommander()
    rarm = MoveGroupCommander("arm_right")

    # get current status
    rarm_initial_pose = rarm.get_current_pose().pose
    rarm_initial_joint_values = rarm.get_current_joint_values()
    
    # set maximum velocity and acceleration
    rarm.set_max_velocity_scaling_factor(0.8)
    rarm.set_max_acceleration_scaling_factor(0.8)

    # right arm pose above object
    p1_jvs = [3.814509, -0.300463, -2.478898, 0.001337, -1.326666, -1.775833, 1.812151]
    # right arm pose to grasp
    p2_jvs = [4.072096, -0.224721, -2.261118, -0.141035, -1.462992, -1.302057, 1.862357]
    # right arm pose above target place to release
    p3_jvs = [0.0, math.radians(40.0), 0.0, math.radians(110.0), 0.0, math.radians(100.0), 0.0]
    # right arm pose to release
    p4_jvs = [0.0, math.radians(50.0), 0.0, math.radians(20.0), 0.0, 0.0, 0.0]

    # execute wiggle motions repeating 2 times for both arms
    rospy.loginfo("Start right arm pick-and-toss motions.")
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

        rospy.loginfo("Moving to tossing start pose...")
        go_with_joint_values(rarm, p3_jvs)
        show_joint_values(rarm)

        rospy.loginfo("Moving to tossing end pose...")
        go_with_joint_values(rarm, p4_jvs)
        show_joint_values(rarm)
        cnt += 1

    # initialize joints
    rospy.loginfo("Initializing both arm joint values...")
    rarm.set_joint_value_target(rarm_initial_joint_values)
    rarm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        pick_and_toss()
    except rospy.ROSInterruptException:
        pass

