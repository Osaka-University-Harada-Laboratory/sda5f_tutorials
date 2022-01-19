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


def wiggle():
    """Executes wiggle motions."""
    rospy.init_node("moveit_command_sender")
    robot = RobotCommander()
    rarm = MoveGroupCommander("arm_right")
    
    # get current status
    larm = MoveGroupCommander("arm_left")
    rarm_initial_pose = rarm.get_current_pose().pose
    rarm_initial_joint_values = rarm.get_current_joint_values()
    larm_initial_pose = larm.get_current_pose().pose
    larm_initial_joint_values = larm.get_current_joint_values()

    # set maximum velocity and acceleration
    rarm.set_max_velocity_scaling_factor(1.0)
    rarm.set_max_acceleration_scaling_factor(1.0)
    larm.set_max_velocity_scaling_factor(1.0)
    larm.set_max_acceleration_scaling_factor(1.0)

    # generate poses
    for_pose_r = Pose()
    for_pose_r.position.x = rarm_initial_pose.position.x + 0.1
    for_pose_r.position.y = rarm_initial_pose.position.y
    for_pose_r.position.z = rarm_initial_pose.position.z - 0.1
    current_euler = quaternion_to_euler(rarm_initial_pose.orientation)
    for_euler = Vector3()
    for_euler.x = current_euler.x + math.radians(20.0) 
    for_euler.y = current_euler.y 
    for_euler.z = current_euler.z
    for_pose_r.orientation = euler_to_quaternion(for_euler)

    back_pose_r = Pose()
    back_pose_r.position.x = rarm_initial_pose.position.x - 0.1
    back_pose_r.position.y = rarm_initial_pose.position.y
    back_pose_r.position.z = rarm_initial_pose.position.z - 0.1
    current_euler = quaternion_to_euler(rarm_initial_pose.orientation)
    back_euler = Vector3()
    back_euler.x = current_euler.x + math.radians(-20.0) 
    back_euler.y = current_euler.y 
    back_euler.z = current_euler.z
    back_pose_r.orientation = euler_to_quaternion(back_euler)

    for_pose_l = Pose()
    for_pose_l.position.x = larm_initial_pose.position.x + 0.1
    for_pose_l.position.y = larm_initial_pose.position.y
    for_pose_l.position.z = larm_initial_pose.position.z - 0.1
    current_euler = quaternion_to_euler(larm_initial_pose.orientation)
    for_euler = Vector3()
    for_euler.x = current_euler.x + math.radians(-20.0) 
    for_euler.y = current_euler.y 
    for_euler.z = current_euler.z
    for_pose_l.orientation = euler_to_quaternion(for_euler)

    back_pose_l = Pose()
    back_pose_l.position.x = larm_initial_pose.position.x - 0.1
    back_pose_l.position.y = larm_initial_pose.position.y
    back_pose_l.position.z = larm_initial_pose.position.z - 0.1
    current_euler = quaternion_to_euler(larm_initial_pose.orientation)
    back_euler = Vector3()
    back_euler.x = current_euler.x + math.radians(20.0) 
    back_euler.y = current_euler.y 
    back_euler.z = current_euler.z
    back_pose_l.orientation = euler_to_quaternion(back_euler)

    # execute wiggle motions repeating 2 times for both arms
    rospy.loginfo("Start right arm wiggle motions...")
    cnt = 0
    while cnt < 2:
        rospy.loginfo("Right arm is moving...")
        rarm.set_pose_target(for_pose_r)
        rarm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        rarm.set_pose_target(back_pose_r)
        rarm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        cnt += 1

    rospy.loginfo("Start left arm wiggle motions...")
    cnt = 0
    while cnt < 2:
        rospy.loginfo("Left arm is moving...")
        larm.set_pose_target(for_pose_l)
        larm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        larm.set_pose_target(back_pose_l)
        larm.go()
        rospy.sleep(rospy.Duration.from_sec(1))
        cnt += 1

    # initialize joints
    rospy.loginfo("Initializing both arm joint values...")
    rarm.set_joint_value_target(rarm_initial_joint_values)
    rarm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    larm.set_joint_value_target(larm_initial_joint_values)
    larm.go()
    rospy.sleep(rospy.Duration.from_sec(1))
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        wiggle()
    except rospy.ROSInterruptException:
        pass

