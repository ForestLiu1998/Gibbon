#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>




bool gripper_close(bool show_in_rviz = true, double gripper_move = 0.0001)
{
    // get the planning gruop of the robot
  moveit::planning_interface::MoveGroupInterface hand_move_group("hand");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// ROS_INFO_NAMED("ros_control", "Planning frame: %s", move_group.getPlanningFrame().c_str());

	// get the joint group of the robot
  std::vector<double> group_variable_values;
  hand_move_group.getCurrentState()->copyJointGroupPositions(
    hand_move_group.getCurrentState()->getRobotModel()->
      getJointModelGroup(hand_move_group.getName()), group_variable_values);

  group_variable_values[0] = gripper_move;
  group_variable_values[1] = gripper_move;
  hand_move_group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
  bool success = (hand_move_group.plan(hand_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("ros_control", "Planning gripper close %s", success ? "SCUUEED" : "FAILED");

  // Visualize the target and the trajection
  if (show_in_rviz)
  {
    const robot_state::JointModelGroup* joint_model_group =
      hand_move_group.getCurrentState()->getJointModelGroup("hand");
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(hand_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }
	
  // move the manipulation along the trajectory
  if (success)
    hand_move_group.execute(hand_plan);
  
  return success;
}



bool gripper_open(bool show_in_rviz = true, double gripper_move = 0.04)
{
    // get the planning gruop of the robot
  moveit::planning_interface::MoveGroupInterface hand_move_group("hand");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// ROS_INFO_NAMED("ros_control", "Planning frame: %s", move_group.getPlanningFrame().c_str());

	// get the joint group of the robot
  std::vector<double> group_variable_values;
  hand_move_group.getCurrentState()->copyJointGroupPositions(
    hand_move_group.getCurrentState()->getRobotModel()->
      getJointModelGroup(hand_move_group.getName()), group_variable_values);

  group_variable_values[0] = gripper_move;
  group_variable_values[1] = gripper_move;
  hand_move_group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
  bool success = (hand_move_group.plan(hand_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("ros_control", "Planning gripper close %s", success ? "SCUUEED" : "FAILED");

  // Visualize the target and the trajection
  if (show_in_rviz)
  {
    const robot_state::JointModelGroup* joint_model_group =
      hand_move_group.getCurrentState()->getJointModelGroup("hand");
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(hand_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }
	
  // move the manipulation along the trajectory
  if (success)
    hand_move_group.execute(hand_plan);
  
  return success;
}




bool manipulator_control(geometry_msgs::Pose& target_pose, bool show_in_rviz = true)
{
  // get the planning gruop of the robot
  moveit::planning_interface::MoveGroupInterface manipulator_move_group("manipulator");
	// ROS_INFO_NAMED("ros_control", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	// ROS_INFO_NAMED("ros_control", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // set the target pose
  manipulator_move_group.setPoseTarget(target_pose);

  // plan the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
  bool success = (manipulator_move_group.plan(manipulator_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("ros_control", "manipulator plan %s", success ? "SCUUEED" : "FAILED");

  // Visualize the target and the trajection
  if (show_in_rviz)
  {
    const robot_state::JointModelGroup* joint_model_group =
      manipulator_move_group.getCurrentState()->getJointModelGroup("manipulator");
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(manipulator_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }
  
  // move the manipulation along the trajectory
  if (success)
    manipulator_move_group.execute(manipulator_plan);
  
  return success;
}

bool manipulator_reset(bool show_in_rviz = true)
{
  // get the planning gruop of the robot
  moveit::planning_interface::MoveGroupInterface manipulator_move_group("manipulator");
	// ROS_INFO_NAMED("ros_control", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	// ROS_INFO_NAMED("ros_control", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // set the zero pose
  geometry_msgs::Pose pose_zero;
  pose_zero.orientation.x = 0.0;
  pose_zero.orientation.y = 0.0;
  pose_zero.orientation.z = 0.0;
	pose_zero.orientation.w = 1.0;
 
  pose_zero.position.x = 0.374000;
  pose_zero.position.y = 0.000000;
  pose_zero.position.z = 0.641000;
  manipulator_move_group.setPoseTarget(pose_zero);

  // plan the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
  bool success = (manipulator_move_group.plan(manipulator_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("ros_control", "manipulator plan %s", success ? "SCUUEED" : "FAILED");

  // Visualize the target and the trajection
  if (show_in_rviz)
  {
    const robot_state::JointModelGroup* joint_model_group =
      manipulator_move_group.getCurrentState()->getJointModelGroup("manipulator");
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(manipulator_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }
  
  // move the manipulation along the trajectory
  if (success)
    manipulator_move_group.execute(manipulator_plan);
  
  bool hand_success = gripper_close(show_in_rviz, 0.0001);

  return success && hand_success;
}



void get_end_effector_pos()
{
  // get the planning gruop of the robot
  moveit::planning_interface::MoveGroupInterface manipulator_move_group("manipulator");
  geometry_msgs::PoseStamped current_end_pose;
	current_end_pose = manipulator_move_group.getCurrentPose("tool0");
	ROS_INFO_NAMED("ros_control", "End effector link's position is (%f, %f, %f)", 
			current_end_pose.pose.position.x, current_end_pose.pose.position.y, current_end_pose.pose.position.z );
	ROS_INFO_NAMED("ros_control", "End effector link's orientation is (%f, %f, %f, %f)", 
			current_end_pose.pose.orientation.x, current_end_pose.pose.orientation.y, current_end_pose.pose.orientation.z, 
			current_end_pose.pose.orientation.w );
}

void add_collusion()
{
  moveit::planning_interface::MoveGroupInterface manipulator_move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = manipulator_move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // add the collision object into the world
  ROS_INFO_NAMED("ros_control", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot_demo", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1); // create a async thread
  spinner.start();


  // set the robot target position
  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = 0.0;
  pose_target.orientation.y = 0.0;
  pose_target.orientation.z = 0.0;
	pose_target.orientation.w = 1.0;
 
  pose_target.position.x = 0.4000;
  pose_target.position.y = 0.000000;
  pose_target.position.z = 0.500;
  
  manipulator_control(pose_target);
 

  gripper_open();


 
  ros::shutdown();


  return 0;
}