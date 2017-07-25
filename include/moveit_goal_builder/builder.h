#ifndef _MOVEIT_GOAL_BUILDER_BUILDER_H_
#define _MOVEIT_GOAL_BUILDER_BUILDER_H_

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/MoveGroupGoal.h"
#include "moveit_msgs/OrientationConstraint.h"
#include "moveit_msgs/WorkspaceParameters.h"

namespace moveit_goal_builder {
enum ActiveTargetType { JOINT, POSE, POSITION, ORIENTATION };

const char kRRTConnect[] = "RRTConnectkConfigDefault";

/// \brief A MoveGroup action goal builder.
///
/// This is used to invoke the MoveGroup action through a custom action client
/// rather than through MoveIt's MoveGroup class. The main reason to do so is if
/// you want to poll the progress of the action.
///
/// To use the builder, set either a pose goal or a joint goal, then set all of
/// the planning properties, which are exposed as public variables.
///
/// \b Example:
/// \code
/// moveit_goal_builder::Builder builder("base_link", "arms");
/// builder.AddPoseGoal("r_wrist_roll_link", pose);
/// builder.AddPathOrientationConstraint(orientation_constraint);
/// builder.planning_time = 10.0;
/// builder.num_planning_attempts = 3;
/// builder.can_replan = true;
/// builder.replan_attempts = 3;
///
/// moveit_msgs::MoveGroupGoal goal;
/// builder.Build(&goal);
/// \endcode
class Builder {
 public:
  /// \brief Constructor
  ///
  /// \param[in] planning_frame The planning frame of the robot, according to
  ///   the robot's MoveIt configuration (e.g., "base_link" or "odom_combined".
  /// \param[in] group_name The name of the group to build the goal for (e.g.,
  ///   "arm")
  Builder(const std::string& planning_frame, const std::string& group_name);

  /// \brief Builds the goal
  ///
  /// \param[out] goal The completed goal.
  void Build(moveit_msgs::MoveGroupGoal* goal) const;

  /// \brief Gets the joint goal that was last added to the builder.
  ///
  /// \param[out] joint_goal The most recently added joint goal.
  void GetJointGoal(std::map<std::string, double>* joint_goal) const;

  /// \brief Sets a joint goal.
  ///
  /// Does not check that the joint names are valid or that the values are
  /// within limits. Setting a joint goal overrides any pose goals that were
  /// previously set.
  ///
  /// \param[in] joints A mapping between joint names and joint angle/position
  /// values.
  void SetJointGoal(const std::map<std::string, double>& joints);

  /// \brief Gets the pose goals that were set.
  ///
  /// \param[out] pose_goals A mapping from end-effector name (the end-effector
  ///   link name, e.g., "r_wrist_roll_link") to the pose goal for that
  ///   end-effector. The pose is specified in the planning frame.
  ///
  /// \see AddPoseGoal, SetPoseGoals
  void GetPoseGoals(
      std::map<std::string, geometry_msgs::Pose>* pose_goals) const;

  /// \brief Sets a pose goal, one for each end-effector.
  ///
  /// Specify the end-effector link and the pose it should reach. The pose
  /// should be specified in the planning frame. Setting a pose goal overrides
  /// any previously set orientation goals.
  ///
  /// \param[in] pose_goals A mapping from end-effector name (the end-effector
  ///   link name, e.g., "r_wrist_roll_link") to the pose goal for that
  ///   end-effector. The pose must be specified in the planning frame.
  ///
  /// \see AddPoseGoal
  void SetPoseGoals(
      const std::map<std::string, geometry_msgs::Pose>& pose_goals);

  /// \brief Clears any pose goals that were set.
  void ClearPoseGoals();

  /// \brief Sets a single pose goal for a single end-effector.
  ///
  /// This can be called multiple times with different end-effectors. Calling
  /// this multiple times with the same end-effector will replace the goal for
  /// that end-effector.
  ///
  /// \param[in] ee_link The name of the end-effector link, e.g.,
  ///   "r_wrist_roll_link"
  /// \param[in] goal The pose goal for that end-effector, in the planning
  ///   frame.
  ///
  /// \see SetPoseGoals
  void AddPoseGoal(const std::string& ee_link, const geometry_msgs::Pose& goal);

  /// \brief Returns the path constraints on the goal.
  ///
  /// \param[out] constraints The constraints on the path.
  void GetPathConstraints(moveit_msgs::Constraints* constraints) const;

  /// \brief Sets the path constraints on the goal.
  ///
  /// \param[out] constraints The constraints on the path.
  void SetPathConstraints(const moveit_msgs::Constraints& constraints);

  /// \brief Clears all path constraints for this goal.
  void ClearPathConstraints();

  /// \brief Adds an orientation constraint to this goal.
  ///
  /// \param[in] oc The orientation constraint to add.
  void AddPathOrientationConstraint(
      const moveit_msgs::OrientationConstraint& oc);

  /// \brief Returns the robot's start state.
  ///
  /// \returns The start state for this planning goal.
  robot_state::RobotStatePtr StartState() const;

  /// \brief Sets the robot's start state for planning.
  ///
  /// \param[in] start_state The start state for the plan.
  void SetStartState(const robot_state::RobotState& start_state);

  /// \brief Sets the robot's start state to be its current state.
  void SetStartStateToCurrentState();

  /// MoveIt planning frame (e.g., "odom_combined" or "base_link").
  std::string planning_frame;

  /// Group name, e.g, "arms", "right_arm", etc.
  std::string group_name;

  /// Whether to compute a plan without executing it.
  bool plan_only;

  /// The workspace boundaries of the robot.
  moveit_msgs::WorkspaceParameters workspace_parameters;

  /// The maximum time to spend finding a plan, in seconds.
  double planning_time;

  /// The name of the planner to use.
  std::string planner_id;

  /// The number of times to compute a plan. The shortest plan will be used.
  unsigned int num_planning_attempts;

  /// Scaling factor in (0, 1] for the maximum joint speed.
  double max_velocity_scaling_factor;

  /// Scaling factor in (0, 1] for the maximum joint acceleration.
  double max_acceleration_scaling_factor;

  /// Whether the robot should be allowed to look around.
  bool can_look;

  /// If the plan fails at execution time, allows the robot to replan and
  /// execute automatically.
  bool can_replan;

  /// Maximum number of times to replan.
  int replan_attempts;

  /// Time to wait in between replanning attempts, in seconds..
  double replan_delay;

  /// Tolerance for the joint angles (applies only to joint angle goals), in
  /// radians.
  double joint_tolerance;

  /// Tolerance radius for the position (applies only to pose goals), in meters.
  double position_tolerance;

  /// Angle tolerance from the X/Y/Z axes of the planning frame, in radians.
  /// Applies only to pose goals.
  double orientation_tolerance;

 private:
  // Joint state goal
  std::map<std::string, double> joint_goal_;

  // Pose goal
  std::map<std::string, geometry_msgs::Pose> pose_goals_;

  // Path constraints
  moveit_msgs::Constraints path_constraints_;

  robot_state::RobotStatePtr start_state_;

  ActiveTargetType active_target_;
};
}  // namespace moveit_goal_builder

#endif  // _MOVEIT_GOAL_BUILDER_BUILDER_H_
