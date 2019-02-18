#!/usr/bin/env python

# Import modules
import numpy as np
from visualization_msgs.msg import Marker
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from franka_description.srv import *

# function to load parameters and request PickPlace service
def franka_mover():

    # TODO: Get parameter "/grasp_list" from ros param server and save it to variable grasp_list, grasp_list is a python dictionary data type

    # Loop through each picking item, the name and pose of the picking items is stored in ros parameter "/grasp_list"
    for obj in grasp_list:

        # TODO: get the name of picking item and store it in ROS message type String
        object_name = String()

        # TODO: get the pose of picking item and store it in ROS message type Pose. Please add an offset of 0.13 meter above the items's z value (grasp_list[object_name]['position']['z'] + 0.13).
        pick_pose = Pose()

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(OBJECT_NAME, PICK_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':

    # TODO: ROS node initialization

    # TODO: call the robot mover function defined above

    # TODO: Spin while node is not shutdown
