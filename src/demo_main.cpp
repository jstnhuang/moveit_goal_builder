#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_goal_builder_demo");

  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> client(
      "move_group", true);
  while (ros::ok() && !client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_group server.");
  }

  moveit_goal_builder::Builder builder("base_link", "arms");
  geometry_msgs::Pose r_pose;
  r_pose.position.x = 0.372;
  r_pose.position.y = -0.07;
  r_pose.position.z = 1.11;
  r_pose.orientation.w = 1;
  geometry_msgs::Pose l_pose;
  l_pose.position.x = 0.372;
  l_pose.position.y = 0.07;
  l_pose.position.z = 1.11;
  l_pose.orientation.w = 1;
  builder.AddPoseGoal("l_wrist_roll_link", l_pose);
  builder.AddPoseGoal("r_wrist_roll_link", r_pose);

  moveit_msgs::MoveGroupGoal goal;
  builder.Build(&goal);
  client.sendGoal(goal);

  while (ros::ok() && !client.getState().isDone()) {
    ros::spinOnce();
  }

  actionlib::SimpleClientGoalState state = client.getState();
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    moveit_msgs::MoveGroupResultConstPtr result = client.getResult();
    if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_INFO("Succeeded!");
    } else {
      ROS_ERROR(
          "MoveIt error: %d (see "
          "http://docs.ros.org/api/moveit_msgs/html/msg/MoveItErrorCodes.html)",
          result->error_code.val);
    }
  } else {
    ROS_ERROR("Action error: %s", state.toString().c_str());
  }

  client.cancelAllGoals();

  return 0;
}
