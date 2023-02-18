// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.auto.AutoConstants.*;

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

  public Command createPath(String path, PathConstraints constraints, boolean isFirstSegment) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, constraints);
    return createPathPlannerCommand(trajectory, isFirstSegment);
  }

  public ArrayList<Command> createPaths(
      String pathGroup,
      PathConstraints constraint,
      boolean isFirstSegment,
      PathConstraints... constraints) {
    ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<>(
        PathPlanner.loadPathGroup(pathGroup, constraint, constraints));

    ArrayList<Command> commands = new ArrayList<>();
    for (PathPlannerTrajectory trajectory : trajectories) {
      commands.add(createPathPlannerCommand(trajectory, isFirstSegment));
    }

    return commands;
  }

  private Command createPathPlannerCommand(PathPlannerTrajectory trajectory, boolean isFirstSegment) {
    PIDController xController = new PIDController(kAutoXTranslationP, kAutoXTranslationI, kAutoXTranslationD);
    PIDController yController = new PIDController(kAutoYTranslationP, kAutoYTranslationI, kAutoYTranslationD);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        kAutoThetaControllerP,
        kAutoThetaControllerI,
        kAutoThetaControllerD,
        kAutoThetaControllerConstraints);

    return new PPTrajectoryFollowCommand(
        trajectory,
        xController,
        yController,
        thetaController,
        changeAutosBasedOnAlliance,
        isFirstSegment,
        this.swerveSubsystem);
  }
}
