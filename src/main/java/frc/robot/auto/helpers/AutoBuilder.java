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
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.commands.PPTrajectoryFollowCommand;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class AutoBuilder {
  private SwerveDrive swerveSubsystem;
  private Map<String, Supplier<Command>> eventMap;
  private Map<String, Command> suppliedEventMap = new HashMap<>();

  public AutoBuilder(SwerveDrive swerveSubsystem) {
    this(swerveSubsystem, new HashMap<>());
  }

  public AutoBuilder(SwerveDrive swerveSubsystem, Map<String, Supplier<Command>> eventMap) {
    this.swerveSubsystem = swerveSubsystem;
    this.eventMap = eventMap;
    for (var entry : eventMap.entrySet()) {
      suppliedEventMap.put(entry.getKey(), entry.getValue().get());
    }
  }

  public Command createPath(String path, PathConstraints constraints, boolean isFirstSegment) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, constraints);
    Command startCommand = createCommandFromStopEvent(trajectory.getStartStopEvent());
    Command pathCommand = createPathPlannerCommand(trajectory, isFirstSegment);
    Command endCommand = createCommandFromStopEvent(trajectory.getEndStopEvent());

    return Commands.sequence(startCommand, pathCommand, endCommand);
  }

  public ArrayList<Command> createPaths(String pathGroup, PathConstraints constraint) {
    return createPaths(pathGroup, constraint, constraint);
  }

  public ArrayList<Command> createPaths(
      String pathGroup, PathConstraints constraint, PathConstraints... constraints) {
    ArrayList<PathPlannerTrajectory> trajectories =
        new ArrayList<>(PathPlanner.loadPathGroup(pathGroup, constraint, constraints));
    ArrayList<Command> commands = new ArrayList<>();

    PathPlannerTrajectory firstTrajectory = trajectories.get(0);
    Command start = createCommandFromStopEvent(firstTrajectory.getStartStopEvent());
    commands.add(start.andThen(createPathPlannerCommand(firstTrajectory, true)));
    trajectories.remove(0);

    for (PathPlannerTrajectory trajectory : trajectories) {
      Command stopEvent = createCommandFromStopEvent(trajectory.getStartStopEvent());
      commands.add(stopEvent.andThen(createPathPlannerCommand(trajectory, false)));
    }
    if (trajectories.size() > 0) {
      PathPlannerTrajectory lastTrajectory = trajectories.get(trajectories.size() - 1);
      commands.add(createCommandFromStopEvent(lastTrajectory.getEndStopEvent()));
    }

    return commands;
  }

  public Command createPathPlannerCommand(
      PathPlannerTrajectory trajectory, boolean isFirstSegment) {
    return createPathPlannerCommand(trajectory, isFirstSegment, changeAutosBasedOnAlliance);
  }

  public Command createPathPlannerCommand(
      PathPlannerTrajectory trajectory, boolean isFirstSegment, boolean useAllianceColor) {

    return new FollowPathWithEvents(
        createTrajectoryFollowCommand(trajectory, isFirstSegment, useAllianceColor),
        trajectory.getMarkers(),
        suppliedEventMap);
  }

  public Command createTrajectoryFollowCommand(
      PathPlannerTrajectory trajectory, boolean isFirstSegment, boolean useAllianceColor) {
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

    return new PPTrajectoryFollowCommand(
        trajectory,
        xTranslationController,
        yTranslationController,
        thetaController,
        useAllianceColor,
        isFirstSegment,
        this.swerveSubsystem);
  }

  public Command createCommandFromStopEvent(StopEvent stopEvent) {
    if (stopEvent.names.isEmpty()) {
      return Commands.waitSeconds(stopEvent.waitTime);
    }

    Command eventCommands = getStopEventCommands(stopEvent);

    switch (stopEvent.waitBehavior) {
      case BEFORE:
        return Commands.sequence(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
      case AFTER:
        return Commands.sequence(eventCommands, Commands.waitSeconds(stopEvent.waitTime));
      case DEADLINE:
        return Commands.deadline(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
      case MINIMUM:
        return Commands.parallel(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
      case NONE:
      default:
        return eventCommands;
    }
  }

  protected Command getStopEventCommands(StopEvent stopEvent) {
    List<Command> commands = new ArrayList<>();

    int startIndex = stopEvent.executionBehavior == ExecutionBehavior.PARALLEL_DEADLINE ? 1 : 0;
    for (int i = startIndex; i < stopEvent.names.size(); i++) {
      String name = stopEvent.names.get(i);
      if (eventMap.containsKey(name)) {
        commands.add(eventMap.get(name).get());
      }
    }

    switch (stopEvent.executionBehavior) {
      case SEQUENTIAL:
        return Commands.sequence(commands.toArray(Command[]::new));
      case PARALLEL:
        return Commands.parallel(commands.toArray(Command[]::new));
      case PARALLEL_DEADLINE:
        Command deadline =
            eventMap.containsKey(stopEvent.names.get(0))
                ? eventMap.get(stopEvent.names.get(0)).get()
                : Commands.none();
        return Commands.deadline(deadline, commands.toArray(Command[]::new));
      default:
        throw new IllegalArgumentException(
            "Invalid stop event execution behavior: " + stopEvent.executionBehavior);
    }
  }
}
