# MoveIt! Humanoid Stability

Center of mass stability predicate for robots with one or more fixed feet based on KDL: center of mass, (static) stability, and support polygon computations.

Mostly a simple wrapper for [hrl_kinematics](https://github.com/ahornung/hrl_kinematics).

Developed by [Dave Coleman](http://dav.ee) and Shintaro Noda at JSK, University of Tokyo.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_humanoid_stability/hydro-devel/resources/screenshot.png" />

## Usage

You can use this validator within a MoveIt! planning scene as a custom stability checker - see the function ``getStateFeasibilityFn()`` for use with a planning scene by calling:
```
planning_scene_->setStateFeasibilityPredicate(humanoid_stability_->getStateFeasibilityFn());
```

## Method

Performs two course-grain checks and, if they pass, finally does full COM check:

 # Checks for a course-grain stability check by seeing if the torso is within a bounding box. This bound box is set in the yaml file and should be found through many iterations. A 0.1m expansion is added to the bounding box so that it over-accepts poses.
 # Checks if the other foot is above ground (z >= 0)
 # Checks if the COM is within the foot projection using hrl_kinematics

It does not check for self collision or collision with environment, because that is accomplished by other components in MoveIt! automatically.

## Configuration

A yaml configuration file is required to load the settings for your robot. See config/hrp2jsknt_stability.yaml

## Limitations / Future Work

 - Improve conversion of joint states from MoveIt! to KDL
 - Smartly check which foot/feet to check for stability
 - Remove assumptions about only being on one foot 
 - Improve other foot check for above ground to include all of mesh, not just the origin of the tip link of the leg