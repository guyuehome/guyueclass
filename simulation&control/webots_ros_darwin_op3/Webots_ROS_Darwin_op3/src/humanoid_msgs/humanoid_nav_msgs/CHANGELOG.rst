^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package humanoid_nav_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2014-01-16)
------------------
* Add service to (re)plan between feet as start and goal.
* Contributors: Armin Hornung

0.2.0 (2013-10-25)
------------------
* Initial catkinization

0.1.2 (2013-01-10)
------------------
* spelling mistake corrected
* added more details to PlanFootsteps srv result
* action ExecFootsteps can now feedback changeable_footsteps and robot_pose (see naoqi docu for further info)
* integrated a new action to communicate with the action server provided by nao_footsteps.py in the nao_driver package
* service to clip footsteps
* moved humanoid_nav_msgs into new humanoid_msgs stack
