// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.AutoConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.DefaultArmElevatorDriveConfig;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoChooser;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoPaths {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private HashMap<String, Command> autoEventMap = new HashMap<>();

  public AutoPaths(
      SwerveDrive swerveSubsystem,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public void sendCommandsToChooser() {
    AutoChooser.createSinglePath("Do Nothing", new InstantCommand());

    if (swerveSubsystem == null
        || intakeSubsystem == null
        || armSubsystem == null
        || elevatorSubsystem == null) {
      AutoChooser.sendChooserToDashboard("Auto Chooser");
      return;
    }

    autoEventMap.put(
        "defaultPosition",
        new ParallelCommandGroup(
            new DefaultArmElevatorDriveConfig(elevatorSubsystem, armSubsystem),
            new IntakeOff(intakeSubsystem)));
    autoEventMap.put(
        "intakeCone",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
            new SetArmAngle(armSubsystem, ArmConstants.kArmGroundIntakeRotation),
            new IntakeCone(intakeSubsystem)));
    autoEventMap.put(
        "intakeCube",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
            new SetArmAngle(armSubsystem, ArmConstants.kArmGroundIntakeRotation),
            new IntakeCube(intakeSubsystem)));
    autoEventMap.put(
        "cubeHigh",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH),
            new SetArmAngle(armSubsystem, ArmConstants.kArmHighRotation),
            new WaitCommand(2).andThen(new IntakeCone(intakeSubsystem))));
    autoEventMap.put(
        "coneHigh",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH),
            new SetArmAngle(armSubsystem, ArmConstants.kArmHighRotation),
            new WaitCommand(2).andThen(new IntakeCube(intakeSubsystem))));
    autoEventMap.put(
        "cubeMid",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID),
            new SetArmAngle(armSubsystem, ArmConstants.kArmMidRotation),
            new WaitCommand(2).andThen(new IntakeCone(intakeSubsystem))));
    autoEventMap.put(
        "coneMid",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID),
            new SetArmAngle(armSubsystem, ArmConstants.kArmMidRotation),
            new WaitCommand(2).andThen(new IntakeCube(intakeSubsystem))));
    autoEventMap.put(
        "cubeLow",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW),
            new SetArmAngle(armSubsystem, ArmConstants.kArmLowRotation),
            new WaitCommand(2).andThen(new IntakeCone(intakeSubsystem))));
    autoEventMap.put(
        "coneLow",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW),
            new SetArmAngle(armSubsystem, ArmConstants.kArmLowRotation),
            new WaitCommand(2).andThen(new IntakeCube(intakeSubsystem))));

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem, autoEventMap);
    Supplier<Command> scorePreload = () -> new ParallelCommandGroup(
        new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH),
        new SetArmAngle(armSubsystem, ArmConstants.kArmHighRotation),
        new WaitCommand(2).andThen(new IntakeCone(intakeSubsystem)));

    // Node5-Engage
    Command node5Engage = autoBuilder
        .createPath("Node5-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node5-Engage", node5Engage);

    // Node8-Preload-Ready
    Command node8PreloadReady = autoBuilder
        .createPath("Node8-Preload-Ready", kDefaultPathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Preload-Ready", node8PreloadReady);

    // Node5-Mobility-Engage
    Command node5MobilityEngage = autoBuilder
        .createPath("Node5-Mobility-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node5-Mobility-Engage", node5MobilityEngage);

    // Node8x2-Engage
    ArrayList<Command> node8x2Engage = autoBuilder.createPaths("Node8x2-Engage", kDefaultPathConstraints,
        kEngagePathConstraints);
    AutoChooser.addIncrementalPaths(scorePreload.get(), "Node8x2-Engage", node8x2Engage);

    // Node2x2-Engage
    ArrayList<Command> node2x2Engage = autoBuilder.createPaths("Node2x2-Engage", kDefaultPathConstraints,
        kEngagePathConstraints);
    AutoChooser.addIncrementalPaths(scorePreload.get(), "Node2x2-Engage", node2x2Engage);

    // Node2-Preload-Ready
    Command node2PreloadReady = autoBuilder
        .createPath("Node2-Preload-Ready", kDefaultPathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Preload-Ready", node2PreloadReady);

    // Node8-Preload-Engage
    Command node8PreloadEngage = autoBuilder
        .createPath("Node8-Preload-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Preload-Engage", node8PreloadEngage);

    // Node8x3-Engage
    // TODO: make name better
    // ArrayList<Command> node8x3Engage = autoBuilder.createPaths(
    // "Node8x3-Engage",
    // kDefaultPathConstraints,
    // kDefaultPathConstraints,
    // kEngagePathConstraints);
    // AutoChooser.addIncrementalPaths(scorePreload.get(), "Node8x3-Engage",
    // node8x3Engage);

    // Node8-Mobility-Engage
    Command node8MobilityEngage = autoBuilder
        .createPath("Node8-Mobility-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Mobility-Engage", node8MobilityEngage);

    // Node2-Preload-Engage
    Command node2PreloadEngage = autoBuilder
        .createPath("Node2-Preload-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Preload-Engage", node2PreloadEngage);

    // Node8x2-Ready
    ArrayList<Command> node8x2Ready = autoBuilder.createPaths("Node8x2-Ready", kDefaultPathConstraints);
    AutoChooser.addIncrementalPaths(scorePreload.get(), "Node8x2-Ready", node8x2Ready);

    // Node2x3-Engage
    // ArrayList<Command> node2x3Engage = autoBuilder.createPaths(
    // "Node2x3-Engage",
    // kDefaultPathConstraints,
    // kDefaultPathConstraints,
    // kEngagePathConstraints);
    // AutoChooser.addIncrementalPaths(scorePreload.get(), "Node2x3-Engage",
    // node2x3Engage);

    // Node2-Engage
    Command node2Engage = autoBuilder
        .createPath("Node2-Engage", kEngagePathConstraints, true)
        .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Engage", node2Engage);
    AutoChooser.sendChooserToDashboard("Auto Chooser");
  }

  public Command getSelectedPath() {
    return AutoChooser.getCommand();
  }
}
