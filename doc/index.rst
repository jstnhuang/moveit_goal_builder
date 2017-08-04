moveit_goal_builder
===================

Goal builder for MoveIt's MoveGroup action.

This is useful for when you want to use the MoveGroup actionlib interface directly, instead of using the MoveGroup class.
This allows you to poll when the action is done, which the normal MoveGroup interface does not allow you to do.

.. toctree::
   :maxdepth: 3

   moveit_goal_builder

Example
=======
To move to a current robot pose with a few options changed.

.. code-block:: python

  import actionlib
  import moveit_msgs
  from moveit_goal_builder import MoveItGoalBuilder

  builder = MoveItGoalBuilder()
  builder.set_pose_goal(pose_stamped)
  builder.replan = True
  builder.replan_attempts = 10
  goal = builder.build()
  
  move_group_action = actionlib.SimpleActionClient('move_group', moveit_msgs.msg.MoveGroupAction)
  move_group_action.send_goal(goal)

See :doc:`moveit_goal_builder` for full API documentation.


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

