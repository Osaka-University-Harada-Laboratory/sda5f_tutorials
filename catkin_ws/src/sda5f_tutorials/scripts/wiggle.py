#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_commander import RobotCommander, MoveGroupCommander
from tf.transformations import quaternion_from_euler, euler_from_quaternion


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
    e = euler_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])


def wiggle():
    """Executes wiggle motions."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()

    # get current status
    larm = MoveGroupCommander("arm_left")
    rarm = MoveGroupCommander("arm_right")

    # left arm joint values for initial position
    initial_left_jvs = [
        1.8457889,
        1.5835086,
        0.,
        1.5871992,
        -1.545645,
        -1.297523,
        0.3945190]
    initial_right_jvs = [
        -1.323263,
        -1.725652,
        0.0099784,
        1.3789879,
        -1.568183,
        1.3330637,
        -0.409884]
    rospy.loginfo("Initializing both arm joints...")
    go_with_joint_values(larm, initial_left_jvs)
    show_joint_values(larm)
    go_with_joint_values(rarm, initial_right_jvs)
    show_joint_values(rarm)

    larm_initial_pose = larm.get_current_pose().pose
    larm_initial_joint_values = larm.get_current_joint_values()
    rarm_initial_pose = rarm.get_current_pose().pose
    rarm_initial_joint_values = rarm.get_current_joint_values()

    # set maximum velocity and acceleration
    larm.set_max_velocity_scaling_factor(0.2)
    larm.set_max_acceleration_scaling_factor(0.2)
    rarm.set_max_velocity_scaling_factor(0.2)
    rarm.set_max_acceleration_scaling_factor(0.2)

    # generate poses
    upper_pose_l = Pose()
    upper_pose_l.position.x = larm_initial_pose.position.x
    upper_pose_l.position.y = larm_initial_pose.position.y
    upper_pose_l.position.z = larm_initial_pose.position.z + 0.1
    current_euler = quaternion_to_euler(larm_initial_pose.orientation)
    upper_euler = Vector3()
    upper_euler.x = current_euler.x + math.radians(-5.0)
    upper_euler.y = current_euler.y
    upper_euler.z = current_euler.z
    upper_pose_l.orientation = euler_to_quaternion(upper_euler)

    lower_pose_l = Pose()
    lower_pose_l.position.x = larm_initial_pose.position.x
    lower_pose_l.position.y = larm_initial_pose.position.y
    lower_pose_l.position.z = larm_initial_pose.position.z + 0.05
    current_euler = quaternion_to_euler(larm_initial_pose.orientation)
    lower_euler = Vector3()
    lower_euler.x = current_euler.x + math.radians(5.0)
    lower_euler.y = current_euler.y
    lower_euler.z = current_euler.z
    lower_pose_l.orientation = euler_to_quaternion(lower_euler)

    upper_pose_r = Pose()
    upper_pose_r.position.x = rarm_initial_pose.position.x
    upper_pose_r.position.y = rarm_initial_pose.position.y
    upper_pose_r.position.z = rarm_initial_pose.position.z + 0.1
    current_euler = quaternion_to_euler(rarm_initial_pose.orientation)
    upper_euler = Vector3()
    upper_euler.x = current_euler.x + math.radians(-5.0)
    upper_euler.y = current_euler.y
    upper_euler.z = current_euler.z
    upper_pose_r.orientation = euler_to_quaternion(upper_euler)

    lower_pose_r = Pose()
    lower_pose_r.position.x = rarm_initial_pose.position.x
    lower_pose_r.position.y = rarm_initial_pose.position.y
    lower_pose_r.position.z = rarm_initial_pose.position.z + 0.05
    current_euler = quaternion_to_euler(rarm_initial_pose.orientation)
    lower_euler = Vector3()
    lower_euler.x = current_euler.x + math.radians(5.0)
    lower_euler.y = current_euler.y
    lower_euler.z = current_euler.z
    lower_pose_r.orientation = euler_to_quaternion(lower_euler)

    # execute wiggle motions using left arm
    rospy.loginfo("Start both arm wiggle motions...")
    cnt = 0
    while cnt < 2:
        rospy.loginfo("Left arm is moving...")
        larm.set_pose_target(upper_pose_l)
        larm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        larm.set_pose_target(lower_pose_l)
        larm.go()
        rospy.sleep(rospy.Duration.from_sec(1))

        rospy.loginfo("Right arm is moving...")
        rarm.set_pose_target(upper_pose_r)
        rarm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        rarm.set_pose_target(lower_pose_r)
        rarm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        cnt += 1

    # initialize joints
    rospy.loginfo("Initializing both arms joint values...")
    larm.set_joint_value_target(larm_initial_joint_values)
    larm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    rarm.set_joint_value_target(rarm_initial_joint_values)
    rarm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        wiggle()
    except rospy.ROSInterruptException:
        pass
