^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmc_collision_detecting_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2025-12-04)
-------------------
* Adjust and implement interfaces to remove robot-specific dependencies.
* Add log message for inconsistent collision check results.
* Implement escape from initial collision state
* Fix the issue where the first feasible trajectory is sometimes not selected when multiple feasible trajectories are available
* Visualize lenear constraints
* Fix typo
* Enable multithreading for validation
* Separate the subscription part for environment information
* Fix the issue where test results were unstable due to random intermediate state
* Remove unused SampleGoalGeneratorMultiThread
* Fix test input for HardPathLinkConstraintsTest
* Update generation condition management
* Add InitialMotionLinearConstraint
* Update interpolation functions for initial_motion_linear_constraint
* Add conversion fucnction for initial_motion_linear_constraint
* Modify the threshold for speed tests for CI
* Use random engine explicitly
* Fully parallelize the trajectory candidates generation process
* Contributors: Keisuke Takeshita

2.0.0 (2024-10-15)
-------------------
* Initial release
* Contributors: Keisuke Takeshita, Satoru Onoda

