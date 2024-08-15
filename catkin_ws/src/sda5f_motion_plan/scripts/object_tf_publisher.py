#!/usr/bin/env python3

import math
import os.path as osp

import tf
import rospy
import rospkg
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor


def update_object_tf(br, scene):
    """Updates an object tf."""
    names = scene.get_known_object_names()
    poses = scene.get_object_poses(names)
    br.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "world",
        "base_link")
    for name, pose in poses.items():
        br.sendTransform(
            (pose.position.x,
             pose.position.y,
             pose.position.z),
            (pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w),
            rospy.Time.now(),
            name,
            "base_link")


def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion
    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def gen_pose(origin, xyz=Vector3(0., 0., 0.), rpy=Vector3(0., 0., 0.)):
    """Generates a pose
    origin: frame_id (string)
    xyz: geometry_msgs/Vector3
    rpy: geometry_msgs/Vector3
    pose: geometry_msgs/PoseStamped
    """
    pose = PoseStamped()
    pose.header.frame_id = origin
    pose.pose.position = xyz
    pose.pose.orientation = euler_to_quaternion(rpy)
    return pose


def gen_color(name, r, g, b, a=1.0):
    """Generates a color
    name: id (string)
    r, g, b, a: float values
    color: moveit_msgs/ObjectColor
    """
    color = ObjectColor()
    color.id = name
    color.color.r = r
    color.color.g = g
    color.color.b = b
    color.color.a = a
    return color


def add_collisions(scene, origin):
    """Updates an object tf."""
    pkg_path = rospkg.RosPack().get_path('sda5f_motion_plan')

    scene.remove_world_object(container_name)
    scene.add_mesh(container_name,
                   gen_pose(origin,
                            Vector3(0.6, 0.18, -0.40),
                            Vector3(0., 0., math.pi/2.)),
                   filename=osp.join(pkg_path,
                                     "meshes",
                                     container_name+".stl"))
    scene.remove_world_object(rack_name)
    scene.add_mesh(rack_name,
                   gen_pose(origin,
                            Vector3(0.2, -0.78, -0.36),
                            Vector3(0., 0., 0.)),
                   filename=osp.join(pkg_path,
                                     "meshes",
                                     rack_name+".stl"))


def scene_tf_publish():
    rospy.init_node("object_tf_publisher")
    planning_scene_interface = PlanningSceneInterface()
    br = tf.TransformBroadcaster()
    rospy.sleep(rospy.Duration(1.0))  # necessary

    add_collisions(planning_scene_interface, 'base_link')
    r = rospy.Rate(60)
    rospy.get_published_topics('/')
    scene_pub = rospy.Publisher(
        'planning_scene', PlanningScene, queue_size=100)
    p = PlanningScene()
    p.is_diff = True
    p.object_colors.append(gen_color(container_name, 0.9, 0.9, 0.9, 0.5))
    p.object_colors.append(gen_color(rack_name, 0.9, 0.9, 0.9, 0.5))

    while not rospy.is_shutdown():
        update_object_tf(br, planning_scene_interface)
        scene_pub.publish(p)
        r.sleep()


if __name__ == '__main__':
    container_name = "container"
    rack_name = "rack"
    try:
        scene_tf_publish()
    except rospy.ROSInterruptException:
        pass
