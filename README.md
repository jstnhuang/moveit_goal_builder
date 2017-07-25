moveit_goal_builder {#mainpage}
===================

Goal builder for MoveIt's MoveGroup action.

This is useful for when you want to use the MoveGroup actionlib interface directly, instead of using the MoveGroup class.
This allows you to poll when the action is done, which the normal MoveGroup interface does not allow you to do.
In the demo linked below, for example, you can stop a MoveIt execution by shutting down the demo node with Ctrl+C.

- [Demo (for PR2)](https://github.com/jstnhuang/moveit_goal_builder/blob/indigo-devel/src/demo_main.cpp)

## C++ example

~~~{.cpp}
moveit_goal_builder::Builder builder("base_link", "arms");
builder.AddPoseGoal("r_wrist_roll_link", pose);
builder.AddPathOrientationConstraint(orientation_constraint);
builder.planning_time = 10.0;
builder.num_planning_attempts = 3;
builder.can_replan = true;
builder.replan_attempts = 3;

moveit_msgs::MoveGroupGoal goal;
builder.Build(&goal);
~~~

## Python example
~~~{.py}
# To move to a current robot pose with a few options changed.
builder = MoveItGoalBuilder()
builder.set_pose_goal(pose_stamped)
builder.replan = True
builder.replan_attempts = 10
goal = builder.build()
~~~

## C++ vs. Python
The C++ and Python APIs are similar, but with a few differences:
- The C++ version supports multi-arm goals, the Python version does not.
- The Python version takes PoseStamped messages for the pose goal and transforms them when you call `build`.
  The C++ version takes Pose messages, which are assumed to be in the planning frame.
