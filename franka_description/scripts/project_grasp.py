#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from franka_description.srv import *
from rospy_message_converter import message_converter
import yaml
import time

import message_filters
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import BoundingBox
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import tf2_ros
import tf2_geometry_msgs

#height: 576
#width: 1024
#distortion_model: plumb_bob
#K: [549.5941880572811, 0.0, 512.5, 0.0, 549.5941880572811, 288.5, 0.0, 0.0, 1.0]
# gqcnn 640*480

def pcl_callback(pcl_msg):
    pass


def callback(depth, rgb, camera_info):

    bounding_box = BoundingBox()
    bounding_box.minX = 0
    bounding_box.minY = 0
    bounding_box.maxX = 1024
    bounding_box.maxY = 576

    print "*************************************************"
    print ("camera_info", camera_info.K)

    rospy.wait_for_service('plan_gqcnn_grasp')

    try:
        plan_routine = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        resp = plan_routine(rgb, depth, camera_info, bounding_box)
        grasp = resp.grasp
        print "*************************************************"
        print ("Pose: ",resp.grasp.pose)
        print ("Grasp_success_prob: ",resp.grasp.grasp_success_prob)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('world', 'camera_rgb_optical_frame', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print "Service call failed: %s"%e
    pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp, trans)
    print "*************************************************"
    print ("pose_transformed: ",pose_transformed)

    try:
        pr2_mover(pose_transformed)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(pose):
    object_name = String()
    object_name.data = 'soap'

    pick_pose = Pose()
    pick_pose.position.x = pose.pose.position.x
    pick_pose.position.y = pose.pose.position.y
    pick_pose.position.z = pose.pose.position.z + 0.20  #position above the item
    pick_pose.orientation = pose.pose.orientation

    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
        resp = pick_place_routine(object_name, pick_pose)
        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Subscribers
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
    rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
    collison_pub = rospy.Publisher("/pr2/3d_map/points", PointCloud2, queue_size=1)

    ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub, info_sub], 1)
    ts.registerCallback(callback)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
