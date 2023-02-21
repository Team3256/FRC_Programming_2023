# Dynamic Path Generation
### Quick start:
* Use the method DynamicPathFollower.run() to make the robot travel to any wanted pose on SmartDashboard
### How it works:
* DynamicPathFinder gets a list of points and finds the best path from start node to end node
* DynamicPathGenerator uses Finder to create a trajectory for the robot to travel from a given start pose to end pose
* DynamicPathFollower uses Generator to create a trajectory from current robot pose to the wanted node from GUI. Then it schedules a command to make the robot run the route.
* To mirror to red paths (default is blue), we subtract all x coordinates from 16.5 to form x' (including obstacles)
### Tests:
* Follow DynamicPathGenerationTest functions to create your own test case and visualize it.
* In red mode all pose rotations are inverted
### Confusing stuff:
* Translation2D is a point
* Rotation2D is a rotation
* Pose2D is a translation with a rotation
* PathNode is a translation with a list of possible next nodes it can travel to. Also has a isPassage flag.
* Path takes in a list of PathNodes and converts in into a list of waypoints
* Waypoint is a point that can be displayed on PathPlanner
* PathPoint is a point that can be used in a Dynamic Trajectory
### Extremely Confusing stuff:
* Translation2D is a position, not a vector, even though we often use it as a vector. Add the vector to the original position to get the final position.
* Rotation2D is stored as a 2x2 transformation matrix. Therefore its value only ever ranges between (-180,180]
* We do not count path length on passages during rotate interpolation
