# Dynamic Path Generation

### Quick start:
* Use the method DynamicPathFollower.run(sink) to make the robot travel to any desired pose.

### Jargon:
* Src refers to the first node in a path. For example the src of the DPG program is the robot's current pose.
* Presink refers to the node right before the sink in a path. For example the presink of the auto score program is the pose right before the robot goes to intake the game piece.
* Sink refers to the last node in a path. For example the sink of the DPG program is the robot's final pose.
* Bezier is the type of curve we use to interpolate between way points. Here is a link to learn more on how they work: https://en.wikipedia.org/wiki/Bézier_curve.
* Passage refers to the narrow space above the charge station and below the charge station in which our robot travels through.

### How it works:
* Visibility graph:
<img width="1137" alt="Screen Shot 2023-03-20 at 5 26 20 PM" src="https://user-images.githubusercontent.com/72239682/226493704-668a83c7-6823-4f55-9748-c0a251e3bd3e.png">
* These are the possible way points to use to travel between src and sink. They are linked in such a way that travelling between nodes does not crash into any obstacles.
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
### Features:
* Optimal control points to interpolate between consecutive points
* Piecewise control scalar that is large when not in community zone
* Convert path into JSON that PathPlanner can render
* Locked rotation during passages that minimizes the robot's radius
* Fastest path algorithm
* Visibility graph
### Bugs:
* DO NOT add/remove blue or red dynamic path way point nodes OUTSIDE of CreateDynamicPathWayNodes
* DO NOT use anything inside dynamicpathgeneration folder EXCEPT for DynamicPathFollower outside of the folder
* We don't want DPG breaking during competition
