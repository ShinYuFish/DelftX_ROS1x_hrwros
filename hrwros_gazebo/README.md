# Gilbreth Gazebo Package

A Gazebo simulation environment for Gilbreth Project.

### Plugins

The Gilbreth simulation environment utilizes several plugins, slightly modified, from the [osrf_gear][1] package.
These include the conveyor belt simulator, vacuum gripper simulator, and object deletion plugins.

Currently the vacuum gripper plugin operates under a few key conditions:
1. No drop conditions are simulated
2. All objects that contact the gripper and whose contacting surface normal is within ~18 degrees of tool normal vector
will be picked up

### Conveyor Spawner

The conveyor spawner node allows for the random generation of a set of predefined objects onto the conveyor belt. Through
a configuration file, a user can specify a mesh resource file, initial pose, and several placement randomization parameters
for each object that should be spawned onto the conveyor.

The conveyor spawner node currently supports the randomization of 4 parameters:
1. Order in which the parts are spawned onto the conveyor
2. Placement of a part across the width of the conveyor (`lateral_placement_variance` parameter)
3. Orientation of part, normal to the surface of the conveyor (`yaw_placement_variance` parameter)
4. Delay from the nominal spawning period (`spawn_timing_variance` parameter)

The conveyor spawner operates under several important assumptions:
1. The random numbers are generated using the std random library, resulting in a uniform distribution of numbers between 0
and RAND_MAX (32767)
2. Each generated random number is normalized such that it becomes a double floating point number between 0 and 1
3. Each "variance" parameter represents a unilateral tolerance.
    - For example, specifying `lateral_placement_variance = 0.1` means that the spawned object could be spawned anywhere
    between +/- 0.1m of the nominal initial pose (in the direction of the width of the conveyor)
4. The randomization engine can be seeded using the `randomization_seed` parameter for replication of the same sequence of
randomized parameters

[1]: https://bitbucket.org/osrf/ariac/overview
