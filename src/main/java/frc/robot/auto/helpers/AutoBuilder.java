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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.auto.commands.PPTrajectoryFollowCommand;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.Map;

public class AutoBuilder {
  private SwerveDrive swerveSubsystem;
  private Map<String, Command> eventMap;

  public AutoBuilder(SwerveDrive swerveSubsystem, Map<String, Command> eventMap) {
    this.swerveSubsystem = swerveSubsystem;
    this.eventMap = eventMap;
  }

  public Command createPath(String path, PathConstraints constraints, boolean isFirstSegment) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, constraints);
    return createPathPlannerCommand(trajectory, isFirstSegment);
  }

  public ArrayList<Command> createPaths(String pathGroup, PathConstraints constraint) {
    ArrayList<PathPlannerTrajectory> trajectories =
        new ArrayList<>(PathPlanner.loadPathGroup(pathGroup, constraint));
    ArrayList<Command> commands = new ArrayList<>();

    commands.add(createPathPlannerCommand(trajectories.get(0), true));
    trajectories.remove(0);
    for (PathPlannerTrajectory trajectory : trajectories) {
      commands.add(createPathPlannerCommand(trajectory, false));
    }

    return commands;
  }

  public ArrayList<Command> createPaths(
      String pathGroup, PathConstraints constraint, PathConstraints... constraints) {
    ArrayList<PathPlannerTrajectory> trajectories =
        new ArrayList<>(PathPlanner.loadPathGroup(pathGroup, constraint, constraints));
    ArrayList<Command> commands = new ArrayList<>();

    commands.add(createPathPlannerCommand(trajectories.get(0), true));
    trajectories.remove(0);
    for (PathPlannerTrajectory trajectory : trajectories) {
      commands.add(createPathPlannerCommand(trajectory, false));
    }

    return commands;
  }

  public Command createPathPlannerCommand(
      PathPlannerTrajectory trajectory, boolean isFirstSegment) {
    PIDController xTranslationController =
        new PIDController(kAutoXTranslationP, kAutoXTranslationI, kAutoXTranslationD);
    PIDController yTranslationController =
        new PIDController(kAutoYTranslationP, kAutoYTranslationI, kAutoYTranslationD);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            kAutoThetaControllerP,
            kAutoThetaControllerI,
            kAutoThetaControllerD,
            kAutoThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    AutoCommandRunner commandRunner = new AutoCommandRunner(trajectory.getMarkers(), eventMap);

    PPTrajectoryFollowCommand path =
        new PPTrajectoryFollowCommand(
            trajectory,
            xTranslationController,
            yTranslationController,
            thetaController,
            changeAutosBasedOnAlliance,
            isFirstSegment,
            this.swerveSubsystem);
    path.setAutoCommandRunner(commandRunner);

    return new ProxyCommand(path);
  }
}
