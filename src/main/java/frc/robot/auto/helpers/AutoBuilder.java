// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.Constants.AutoConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.commands.PPTrajectoryFollowCommand;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;

public class AutoBuilder {
  // TODO: Add AutoSpec
  private SwerveDrive swerveSubsystem;

  public AutoBuilder(SwerveDrive swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  public Command createPath(String path, PathConstraints constraints) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, constraints);
    return createCommand(trajectory);
  }

  public ArrayList<Command> createPaths(
      String pathGroup, PathConstraints contraint, PathConstraints... constraints) {
    ArrayList<PathPlannerTrajectory> trajectories =
        new ArrayList<>(PathPlanner.loadPathGroup(pathGroup, contraint, constraints));

    ArrayList<Command> commands = new ArrayList<>();
    for (PathPlannerTrajectory trajectory : trajectories) {
      commands.add(createCommand(trajectory));
    }

    return commands;
  }

  private Command createCommand(PathPlannerTrajectory trajectory) {
    PIDController xController = new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER);
    PIDController yController = new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            P_THETA_CONTROLLER,
            I_THETA_CONTROLLER,
            D_THETA_CONTROLLER,
            THETA_CONTROLLER_CONSTRAINTS);

    return new PPTrajectoryFollowCommand(
        trajectory, xController, yController, thetaController, this.swerveSubsystem);
  }
}
