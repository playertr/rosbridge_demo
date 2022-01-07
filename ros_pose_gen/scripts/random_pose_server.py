#!/usr/bin/env python3

from ros_pose_gen.srv import GenRandomPose, GenRandomPoseResponse
from ros_pose_gen.srv import AddTwoInts,AddTwoIntsResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import String
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

#
# Trivial publisher from the ROS beginner tutorial
#
class ChatterPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.chatter)

    def chatter(self,event):
        rospy.loginfo("Chatter!")
        hello_str = "hello world %s" % event.current_real
        self.pub.publish(hello_str)

#
# Trivial service from the ROS beginner tutorial
#
class AddTwoIntsService():
    def __init__(self):
        self.srv = rospy.Service("add_two_ints", AddTwoInts, self.handle_add_two_ints)

    def handle_add_two_ints(self,req):
        return AddTwoIntsResponse(req.a + req.b)



def gen_random_pose_server():
    rospy.init_node('gen_random_pose_server')
    s = rospy.Service('gen_random_pose', GenRandomPose, handle_gen_random_pose)

    chatter = ChatterPublisher()
    add_two_srv = AddTwoIntsService()

    print("Ready to generate random poses.")
    rospy.spin()

if __name__ == "__main__":
    gen_random_pose_server()
