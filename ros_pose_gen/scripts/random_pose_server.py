#!/usr/bin/python

from ros_pose_gen.srv import GenRandomPose, GenRandomPoseResponse
from geometry_msgs.msg import Pose
import rospy
import numpy as np

def handle_gen_random_pose(req):
    print("Generating {} random poses.".format(req.NumPoses))

    ## Make a list with the requested number of "random" poses
    poses = []
    for i in range(req.NumPoses):
        p = Pose()
        p.position.x = np.random.rand()
        p.position.y = np.random.rand()
        p.position.z = np.random.rand()
        # Make sure the quaternion is valid and normalized
        quat = np.random.rand(4)
        quat = quat / np.linalg.norm(quat)
        (
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        ) = quat
        poses.append(p)

    return GenRandomPoseResponse(poses)

def gen_random_pose_server():
    rospy.init_node('gen_random_pose_server')
    s = rospy.Service('gen_random_pose', GenRandomPose, handle_gen_random_pose)
    print("Ready to generate random poses.")
    rospy.spin()

if __name__ == "__main__":
    gen_random_pose_server()
