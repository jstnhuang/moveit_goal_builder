#include "moveit_goal_builder/builder.h"

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/BoundingVolume.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/JointConstraint.h"
#include "moveit_msgs/MoveGroupGoal.h"
#include "moveit_msgs/PositionConstraint.h"
#include "shape_msgs/SolidPrimitive.h"

#include "ros/ros.h"

typedef std::map<std::string, double> JointValues;
typedef std::map<std::string, geometry_msgs::Pose> PoseGoals;

namespace moveit_goal_builder {
Builder::Builder(const std::string& planning_frame,
                 const std::string& group_name)
    : planning_frame(planning_frame),
      group_name(group_name),
      plan_only(false),
      workspace_parameters(),
      planning_time(5.0),
      planner_id(kRRTConnect),
      num_planning_attempts(1),
      max_velocity_scaling_factor(1.0),
      max_acceleration_scaling_factor(1.0),
      can_look(false),
      can_replan(false),
      replan_attempts(3),
      replan_delay(2.0),
      joint_tolerance(1e-4),
      position_tolerance(1e-4),
      orientation_tolerance(1e-3),
      joint_goal_(),
      pose_goals_(),
      start_state_(),
      active_target_(JOINT) {}

void Builder::Build(moveit_msgs::MoveGroupGoal* goal) const {
  goal->request.group_name = group_name;
  goal->request.num_planning_attempts = num_planning_attempts;
  goal->request.max_velocity_scaling_factor = max_velocity_scaling_factor;
  goal->request.max_acceleration_scaling_factor =
      max_acceleration_scaling_factor;
  goal->request.allowed_planning_time = planning_time;
  goal->request.planner_id = planner_id;
  goal->request.workspace_parameters = workspace_parameters;

  if (start_state_) {
    robot_state::robotStateToRobotStateMsg(*start_state_,
                                           goal->request.start_state);
  } else {
    goal->request.start_state.is_diff = true;
  }

  if (active_target_ == JOINT) {
    goal->request.goal_constraints.resize(1);
    moveit_msgs::Constraints c1;
    for (JointValues::const_iterator it = joint_goal_.begin();
         it != joint_goal_.end(); ++it) {
      moveit_msgs::JointConstraint jc;
      jc.joint_name = it->first;
      jc.position = it->second;
      jc.tolerance_above = joint_tolerance;
      jc.tolerance_below = joint_tolerance;
      jc.weight = 1.0;
      c1.joint_constraints.push_back(jc);
    }
    goal->request.goal_constraints[0] = c1;
  } else if (active_target_ == POSE) {
    goal->request.goal_constraints.resize(1);
    moveit_msgs::Constraints& constraint = goal->request.goal_constraints[0];

    for (PoseGoals::const_iterator it = pose_goals_.begin();
         it != pose_goals_.end(); ++it) {
      // Add position constraint
      moveit_msgs::PositionConstraint pc;
      pc.header.frame_id = planning_frame;
      pc.link_name = it->first;
      shape_msgs::SolidPrimitive sp;
      sp.type = shape_msgs::SolidPrimitive::SPHERE;
      sp.dimensions.push_back(position_tolerance);
      moveit_msgs::BoundingVolume bv;
      bv.primitives.push_back(sp);
      bv.primitive_poses.push_back(it->second);
      pc.constraint_region = bv;
      pc.weight = 1.0;
      constraint.position_constraints.push_back(pc);

      moveit_msgs::OrientationConstraint oc;
      oc.header.frame_id = planning_frame;
      oc.link_name = it->first;
      oc.orientation = it->second.orientation;
      oc.absolute_x_axis_tolerance = orientation_tolerance;
      oc.absolute_y_axis_tolerance = orientation_tolerance;
      oc.absolute_z_axis_tolerance = orientation_tolerance;
      oc.weight = 1.0;
      constraint.orientation_constraints.push_back(oc);
    }
  } else {
    ROS_ERROR_NAMED("moveit_goal_builder",
                    "Unable to construct goal representation");
  }

  goal->request.path_constraints = path_constraints_;

  goal->planning_options.plan_only = plan_only;
  goal->planning_options.look_around = can_look;
  goal->planning_options.replan = can_replan;
  goal->planning_options.replan_attempts = replan_attempts;
  goal->planning_options.replan_delay = replan_delay;
  goal->planning_options.planning_scene_diff.is_diff = true;
  goal->planning_options.planning_scene_diff.robot_state.is_diff = true;
}

void Builder::GetJointGoal(std::map<std::string, double>* joint_goal) const {
  *joint_goal = joint_goal_;
}

void Builder::SetJointGoal(const std::map<std::string, double>& joints) {
  active_target_ = JOINT;
  joint_goal_ = joints;
}

void Builder::GetPoseGoals(
    std::map<std::string, geometry_msgs::Pose>* pose_goals) const {
  *pose_goals = pose_goals_;
}

void Builder::SetPoseGoals(
    const std::map<std::string, geometry_msgs::Pose>& pose_goals) {
  active_target_ = POSE;
  pose_goals_ = pose_goals;
}

void Builder::ClearPoseGoals() { pose_goals_.clear(); }

void Builder::AddPoseGoal(const std::string& ee_link,
                          const geometry_msgs::Pose& goal) {
  active_target_ = POSE;
  pose_goals_[ee_link] = goal;
}

void Builder::GetPathConstraints(moveit_msgs::Constraints* constraints) const {
  *constraints = path_constraints_;
}

void Builder::SetPathConstraints(const moveit_msgs::Constraints& constraints) {
  path_constraints_ = constraints;
}

void Builder::ClearPathConstraints() {
  path_constraints_.joint_constraints.clear();
  path_constraints_.orientation_constraints.clear();
  path_constraints_.position_constraints.clear();
  path_constraints_.visibility_constraints.clear();
}

void Builder::AddPathOrientationConstraint(
    const moveit_msgs::OrientationConstraint& oc) {
  path_constraints_.orientation_constraints.push_back(oc);
}

robot_state::RobotStatePtr Builder::StartState() const { return start_state_; }

void Builder::SetStartState(const robot_state::RobotState& start_state) {
  start_state_.reset(new robot_state::RobotState(start_state));
}

void Builder::SetStartStateToCurrentState() { start_state_.reset(); }
}  // namespace moveit_goal_builder
