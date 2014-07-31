# MoveIt! Humanoid Stability

Libraries for sampling and validating a humanoid biped with one or more fixed feet for stability constraints in MoveIt! including:

 - Fast stability heuristic checking
 - Center of mass, static stability, and support polygon computations.
 - Smart sampling of whole body positions using random number generation

Provides a wrapper for [hrl_kinematics](https://github.com/ahornung/hrl_kinematics) that allows the center of mass stability testing for robots based on KDL.

Developed by [Dave Coleman](http://dav.ee) and Shintaro Noda at JSK, University of Tokyo.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_humanoid_stability/hydro-devel/resources/screenshot.png" />

## Install

Making this work on your machine will be fairly complicated until a long list of pending MoveIt! pull requests are accepted. See Dave Coleman for more details.

## Usage and Explanation

### Balance Constraint Validator

Performs two course-grain checks and, if they pass, finally does full COM check:

 1. Checks for a course-grain stability check by seeing if the torso is within a bounding box. This bound box is set in the yaml file and should be found through many iterations. A 0.1m expansion is added to the bounding box so that it over-accepts poses.
 2. Checks if the other foot is above ground (z >= 0)
 3. Checks if the COM is within the foot projection using hrl_kinematics

It does not check for self collision or collision with environment, because that is accomplished by other components in MoveIt! automatically.

You can use the validator within a MoveIt! planning scene as a custom stability checker - see the function ``getStateFeasibilityFn()`` for use with a planning scene by calling:

```
planning_scene_->setStateFeasibilityPredicate(humanoid_stability_->getStateFeasibilityFn());
```

### State Sampling

The sampler first samples the fixed leg, performs some quick checks, then samples the rest of the body and performs exact checks.

You can use a custom constraint sampler plugin for MoveIt! by adding a "constraint_samplers" param to your move_group launch file, for example:

```
  <node name="hrp2jsknt_moveit_demos" pkg="hrp2jsknt_moveit_demos" type="hrp2_demos">
    <rosparam command="load" file="$(find moveit_humanoid_stability)/config/hrp2jsknt_stability.yaml"/>
    <param name="constraint_samplers" value="moveit_humanoid_stability/HumanoidConstraintSamplerAllocator"/>
  </node>
```

## Configuration

A yaml configuration file is required to load the settings for your robot. See [config/hrp2jsknt_stability.yaml](https://github.com/davetcoleman/moveit_humanoid_stability/blob/hydro-devel/config/hrp2jsknt_stability.yaml)

## Limitations / Future Work

### Stability Validator 

 - Only supports standing on left foot
 - Only supports fixed link remaining the same (never changes location or link)
 - Improve conversion of joint states from MoveIt! to KDL
 - Smartly check which foot/feet to check for stability
 - Remove assumptions about only being on one foot 
 - Improve other foot check for above ground to include all of mesh, not just the origin of the tip link of the leg

### Constraint Sampler

 - Remove HRP2JSK assumptions / hard-coded values
 - Cleanup code (currently research code)