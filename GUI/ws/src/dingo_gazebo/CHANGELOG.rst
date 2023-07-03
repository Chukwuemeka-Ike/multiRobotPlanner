^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2023-04-20)
------------------
* Bump CMake version to avoid CMP0048 warning.
* Contributors: Tony Baltovski

0.1.1 (2022-01-18)
------------------
* Remove the realsense-to-laser node, as it's been axed from the core robot_bringup too
* Add an accessories folder for launching additional nodes that would be part of the bringup on a real robot.  Currently populated with the corresponding realsense nodes from dingo_robot
* Enable passing the config argument to dingo_description
* Contributors: Chris Iverach-Brereton

0.1.0 (2020-08-12)
------------------
* Revert to using dirname; i blanked on the conversation where we went over the new standards
* Add the empty world launch file (useful for replaying bags w/o obstacles getting in the way)
* Add spawn_dingo.launch for compatibility with the new simulation environments. Enable teleop by default in the simulations. Use $(find ...) instead of $(dirname) for improved compatibility with external packages
* Unified launch files, added platform specific gains and updated dependencies.
* [dingo_gazebo] Removed media.
* Initial commit for handoff to tbaltovski; still some updates required
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski
