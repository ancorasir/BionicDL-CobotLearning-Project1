import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from std_msgs.msg import String
import rosbag
import numpy as np
from matplotlib import pyplot as plt

NUM_ATTEMPS = 1
PLANNER_NAME = "RRTConnect"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("panda_gripper")

group.get_current_joint_values()

Q=[0.04,0.04]
group.set_joint_value_target(Q)
plan = group.plan()
group.execute(plan)
