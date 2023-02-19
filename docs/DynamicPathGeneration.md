# Dynamic Path Generation
### Quick start:
* Use the method DynamicPathFollower.run() to make the robot travel to the wanted node on SmartDashboard
### How it works:
* DynamicPathFinder gets a list of points and finds the best path from start node to end node
* DynamicPathFollower uses Finder to create a trajectory for the robot to travel
### Confusing stuff:
* Translation2D is a point
* Rotation2D is a rotation
* Pose2D is a translation with a rotation
* PathNode is a translation with a list of possible next nodes it can travel to. Also has a isPassage flag.
* Path takes in a list of PathNodes and converts in into a list of waypoints
* Waypoint is a point that can be displayed on PathPlanner
* PathPoint is a point that can be used in a Dynamic Trajectory
