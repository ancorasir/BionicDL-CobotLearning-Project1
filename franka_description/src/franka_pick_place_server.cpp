/*******************************************************************************
 *
 * This file is part of cobot learning project1
 *
 ******************************************************************************/

// Author: WAN FANG

#include <franka_description/franka_pick_place_server.h>

FRANKAPickPlace::FRANKAPickPlace(ros::NodeHandle nh)
  : nh_(nh),
    move_group(ARM_PLANNING_GROUP),
    gripper_group(GRIPPER_PLANNING_GROUP)
{
  client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
  gripper_joint_model_group =
    gripper_group.getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);

  visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("world"));
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->loadRemoteControl();

  // Create text marker for displaying current state
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.6;
  visual_tools_ptr->publishText(text_pose, "Welcome to Advance Pick and Place project",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);

  // Publish messages to rviz
  visual_tools_ptr->trigger();

  /*
   * Collision Objects:
   * Create an object list and populate it with dropbox objects
   * Then insert objects in scene for collision avoidance and interaction
   */
  std::vector<moveit_msgs::CollisionObject> collision_object_list;
  std::vector<std::string> object_ids;
  moveit_msgs::CollisionObject dropbox_collision_object;

  // Define pose for the objects (specified relative to base_footprint)
  geometry_msgs::Pose mesh_pose;

  mesh_pose.position.x = -0.0;
  mesh_pose.position.y = -0.70;
  mesh_pose.position.z = 0;
  mesh_pose.orientation.w = 0.707;
  mesh_pose.orientation.x = 0;
  mesh_pose.orientation.y = 0;
  mesh_pose.orientation.z = 0.707;

  SetupCollisionObject("dropbox", DROPBOX_MESH_PATH, mesh_pose,
                       dropbox_collision_object);

  collision_object_list.push_back(dropbox_collision_object);

  // Add the object list to the world scene
  planning_scene_interface.addCollisionObjects(collision_object_list);
  ROS_INFO("Added object list to the world");
  ros::Duration(1.0).sleep();

  //Ready to pick
  move_group.setNamedTarget("place_pose");

  success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  OperateGripper(false);
  ros::Duration(5.0).sleep();

}

bool FRANKAPickPlace::Routine(franka_description::PickPlace::Request &req,
                    franka_description::PickPlace::Response &res)
{

  visual_tools_ptr->deleteAllMarkers();

  // Create text marker for displaying current state
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.6;
  visual_tools_ptr->publishText(text_pose, "New request received",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->trigger();

  //Is pick_pose close to actual object pose
  gazebo_msgs::GetModelState srv;
  geometry_msgs::Pose place_pose, grasp_pose;

  grasp_pose = req.pick_pose;

  srv.request.model_name = req.object_name.data;
  srv.request.relative_entity_name = "world";

  // Pick Pose is within limits, next spawn the collision object
  geometry_msgs::Pose target_mesh_pose;
  std::vector<moveit_msgs::CollisionObject> target_object_list;
  std::vector<std::string> object_ids;

  // Plan arm motion
  // set starting pose
  move_group.setStartStateToCurrentState();

  // set target pose
  //Go to the 0.2m above the picking object
  grasp_pose.position.z = grasp_pose.position.z + 0.2;
  move_group.setPoseTarget(grasp_pose);

  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
           success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO("Visualizing plan 1 as trajectory line");
  visual_tools_ptr->publishAxisLabeled(grasp_pose, "reach_pose");
  visual_tools_ptr->publishText(text_pose, "Reach Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();

  move_group.execute(arm_plan);

  //Reach movement
  move_group.setStartStateToCurrentState();
  grasp_pose.position.z = grasp_pose.position.z - 0.2;
  move_group.setPoseTarget(grasp_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
           success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(grasp_pose, "pick_pose");
  visual_tools_ptr->publishText(text_pose, "Pick Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();

  move_group.execute(arm_plan);

  object_ids.push_back(req.object_name.data);

  //Close Gripper
  OperateGripper(true);
  ros::Duration(3.0).sleep();

  //Reach movement
  move_group.setStartStateToCurrentState();
  grasp_pose.position.z = grasp_pose.position.z + 0.2;
  move_group.setPoseTarget(grasp_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
           success ? "SUCCEEDED" : "FAILED");
  // visualize the plan in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(grasp_pose, "reach_pose");
  visual_tools_ptr->publishText(text_pose, "Reach Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();

  move_group.execute(arm_plan);

  //drop the ball
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget("place_pose");
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
           success ? "SUCCEEDED" : "FAILED");

  visual_tools_ptr->publishText(text_pose, "Drop Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  move_group.execute(arm_plan);

  //Open Gripper
  OperateGripper(false);
  ros::Duration(5.0).sleep();

  res.success = true;
}


bool FRANKAPickPlace::SetupCollisionObject(const std::string &object_id,
    const std::string &mesh_path,
    const geometry_msgs::Pose &object_pose,
    moveit_msgs::CollisionObject &collision_object)
{
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

  ROS_DEBUG_STREAM(object_id << " mesh loaded");

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
}

bool FRANKAPickPlace::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    gripper_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.0;
    gripper_joint_positions[1] = 0.0;
  }
  else
  {
    gripper_joint_positions[0] = 0.04;
    gripper_joint_positions[1] = 0.04;
  }

  gripper_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = gripper_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  return success;
}

tf::Quaternion FRANKAPickPlace::RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}

FRANKAPickPlace::~FRANKAPickPlace(){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "franka_pick_place_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(8);
  spinner.start();
  FRANKAPickPlace franka_pick_place(nh);
  ros::ServiceServer service = nh.advertiseService("pick_place_routine", &FRANKAPickPlace::Routine, &franka_pick_place);
  ros::waitForShutdown();
  return 0;
}
