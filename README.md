# moveit_goal_builder

Goal builder for MoveIt's MoveGroup action.

This is useful for when you want to use the MoveGroup action interface directly, instead of using the MoveGroup class.
This allows you to poll when the action is done, which the normal MoveGroup interface does not allow you to do.

- [Demo (for PR2)](src/demo_main.cpp)

## C++ vs. Python
The C++ and Python versions are similar, but with a few differences:
- The C++ version supports multi-arm goals, the Python version does not
- The Python version takes PoseStamped messages for the pose goal and transforms them when you call `build`.
  The C++ version takes Pose messages, which are assumed to be in the planning frame.
