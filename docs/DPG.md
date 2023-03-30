# Dynamic Path Generation

### Quick start:
* Create a new Dynamic Path Generator
  * Ex: DynamicPathGenerator dpg = new DynamicPathGenerator(Pose2d robotPose, Pose2d targetPose)
* Use the method getCommand to get the command to run in order to run the trajectory.
    * Ex: Command cmd = dpg.getCommand(swerveDrive, pathConstraints)
* Run the command
    * Ex: CommandScheduler.run(cmd)

### Simulating quick start:
* Create 2 windows in glass: waypointViewer + Auto Visualization
* Set alliance color in SimulationGUI to wanted color (red or blue).
* Turn teleoperated to on.
* Assign your keyboard to Joystick port 2.
* Press Z to run the DPG trajectory
* Change the initial swerve pose and final swerve pose to observe how the trajectory changes.

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
* Dynamic Path Generator sends this graph to DynamicPathFinder which finds the best path from the start node to end node.
* Dynamic Path Finder uses Djikstra's shortest path algorithm to find the optimal path in the visibility graph to get from src to sink in minimal time.
* Programs like AutoScore and AutoDoubleSubstation uses Dynamic Path Generator to create a trajectory from the current robot pose to the wanted node in SmartDashboard.

### Minor details:
* To mirror to red paths (default is blue), we subtract all blue x coordinates from the field width to get the corresponding red x coordinate. This including the field obstacle objects we created as well. Then we invert the rotation associated with each blue pose.

### Tests:
* Look at the DynamicPathGenerationTest file and its functions to create your own test case and visualize the result. There are many examples available in the associated directory.
* In red mode all the tests becomes red tests instead of blue tests.

### Intermediary classes used:
* Translation2D is a point, sometimes used as a vector
* Rotation2D is a rotation
* Pose2D is a translation with a rotation
* PathNode is a translation with a list of possible next nodes it can travel to. Also has a isPassage flag.
* Path takes in a list of PathNodes and converts in into a list of waypoints
* Waypoint is a point that can be displayed on PathPlanner
* PathPoint is a point that can be used in a Dynamic Trajectory

### Implementation:
* Translation2D is a position, not a vector, even though we often use it as a vector. Add the vector to the original position to get the final position.
* Rotation2D is stored as a 2x2 transformation matrix. Therefore its value only ever ranges between (-180,180]

### Features:
* Optimal control points to interpolate a bezier curve between consecutive points
* Piecewise control scalar that is adaptive to the state of the point. For example, it is extra small when the point is in the community zone, and extra large when the next point is very far away.
* Convert a path or set of points into JSON that the PathPlanner app can render
* Locked rotation during passages that minimizes the robot's radius
* Fastest path algorithm
* Visibility graph
* Integrated with FMS Alliance Color
* Microservice that can be used as a plugin for programs
