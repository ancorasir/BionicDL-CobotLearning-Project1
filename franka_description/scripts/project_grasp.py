#!/usr/bin/env python

import numpy as np
from visualization_msgs.msg import Marker
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from franka_description.srv import *
from rospy_message_converter import message_converter
import time

#height: 576
#width: 1024
#distortion_model: plumb_bob
#K: [549.5941880572811, 0.0, 512.5, 0.0, 549.5941880572811, 288.5, 0.0, 0.0, 1.0]
# gqcnn 640*480
def callback(depth, rgb, camera_info):
    pass

# function to load parameters and request PickPlace service
def franka_mover():
    grasp_list = rospy.get_param('/grasp_list')[0]

    for obj in grasp_list:
        object_name = String()
        object_name.data = 'glue'

        pick_pose = Pose()
        pick_pose.position.x = grasp_list[obj]['position']['x']
        pick_pose.position.y = grasp_list[obj]['position']['y']
        pick_pose.position.z = grasp_list[obj]['position']['z'] + 0.13  #+ offset for gripper from contact point to +
        pick_pose.orientation.x = grasp_list[obj]['orientation']['x']
        pick_pose.orientation.y = grasp_list[obj]['orientation']['y']
        pick_pose.orientation.z = grasp_list[obj]['orientation']['z']
        pick_pose.orientation.w = grasp_list[obj]['orientation']['w']
        print ("Pick pose: ",pick_pose)
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            resp = pick_place_routine(object_name, pick_pose)
            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('franka_picking', anonymous=True)
    franka_mover()

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
