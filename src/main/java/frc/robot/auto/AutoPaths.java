// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.AutoConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoChooser;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerveX;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoPaths {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private HashMap<String, Supplier<Command>> autoEventMap = new HashMap<>();

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
    AutoChooser.createSingleDefaultPath("Do Nothing", new InstantCommand());

    if (swerveSubsystem == null
        || intakeSubsystem == null
        || armSubsystem == null
        || elevatorSubsystem == null) {
      AutoChooser.sendChooserToDashboard("Auto Chooser");
      return;
    }

    autoEventMap.put(
        "defaultPosition",
        () ->
            runParallelWithPath(
                new ParallelCommandGroup(
                        new StowArmElevator(elevatorSubsystem, armSubsystem),
                        new IntakeOff(intakeSubsystem))
                    .withTimeout(2.5)
                    .asProxy()
                    .withName("defaultPosition")));
    autoEventMap.put(
        "intakeCone",
        () ->
            runParallelWithPath(
                new ParallelCommandGroup(
                        new SetElevatorHeight(
                            elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
                        new SetArmAngle(armSubsystem, ArmPosition.GROUND_INTAKE),
                        new IntakeCone(intakeSubsystem))
                    .withTimeout(1.75)
                    .asProxy()
                    .withName("intakeCone")));
    autoEventMap.put(
        "intakeCube",
        () ->
            runParallelWithPath(
                    new ParallelCommandGroup(
                        new SetElevatorHeight(
                            elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
                        new SetArmAngle(armSubsystem, ArmPosition.GROUND_INTAKE),
                        new IntakeCube(intakeSubsystem)))
                .withTimeout(1.75)
                .asProxy()
                .withName("intakeCube"));
    autoEventMap.put(
        "cubeHigh",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CUBE_HIGH),
                    new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
                    new WaitCommand(0.5).andThen(new IntakeCone(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("cubeHigh"));
    autoEventMap.put(
        "coneHigh",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.CONE_HIGH),
                    new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
                    new WaitCommand(0.5).andThen(new IntakeCube(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("coneHigh"));
    autoEventMap.put(
        "cubeMid",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(
                        elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_MID),
                    new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
                    new WaitCommand(0.5).andThen(new IntakeCone(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("cubeMid"));
    autoEventMap.put(
        "coneMid",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(
                        elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_MID),
                    new SetArmAngle(armSubsystem, ArmPosition.CONE_MID),
                    new WaitCommand(0.5).andThen(new IntakeCube(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("coneMid"));
    autoEventMap.put(
        "cubeLow",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(
                        elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_LOW),
                    new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW),
                    new WaitCommand(0.5).andThen(new IntakeCone(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("cubeLow"));
    autoEventMap.put(
        "coneLow",
        () ->
            new ParallelCommandGroup(
                    new SetElevatorHeight(
                        elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_LOW),
                    new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW),
                    new WaitCommand(0.5).andThen(new IntakeCube(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("coneLow"));

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem, autoEventMap);
    Supplier<Command> scorePreload =
        () ->
            new ParallelCommandGroup(
                    new ZeroElevator(elevatorSubsystem),
                    new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
                    new WaitCommand(1.35).andThen(new IntakeCone(intakeSubsystem)))
                .withTimeout(1.5)
                .asProxy()
                .withName("scorePreload");

    Command node8Mobility =
        autoBuilder
            .createPath("Node8-Mobility", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Mobility", node8Mobility);

    Command node2Mobility =
        autoBuilder
            .createPath("Node2-Mobility", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Mobility", node2Mobility);

    // Node5-Engage
    ArrayList<Command> node5Engage =
        autoBuilder.createPaths("Node5-Engage", kEngagePathConstraints);
    AutoChooser.addPathGroup(
        scorePreload.get(), "Node5-Engage", node5Engage, new LockSwerveX(swerveSubsystem));

    // Node8-Preload-Ready
    Command node8PreloadReady =
        autoBuilder
            .createPath("Node8-Preload-Ready", kSafePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Preload-Ready", node8PreloadReady);

    // Node5-Mobility-Engage
    Command node5MobilityEngage =
        autoBuilder
            .createPath("Node5-Mobility-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node5-Mobility-Engage", node5MobilityEngage);

    // Node8x2
    Command node8x2 =
        autoBuilder
            .createPath("Node8x2", kFastPathConstraints, true)
            .beforeStarting(scorePreload.get().asProxy());
    AutoChooser.createSinglePath("Node8x2", node8x2);

    // Node2x2-Engage
    ArrayList<Command> node2x2Engage =
        autoBuilder.createPaths("Node2x2-Engage", kSafePathConstraints, kEngagePathConstraints);
    AutoChooser.addPathGroup(
        scorePreload.get(), "Node2x2-Engage", node2x2Engage, new LockSwerveX(swerveSubsystem));

    // Node2-Preload-Ready
    Command node2PreloadReady =
        autoBuilder
            .createPath("Node2-Preload-Ready", kSafePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Preload-Ready", node2PreloadReady);

    // Node8-Preload-Engage
    Command node8PreloadEngage =
        autoBuilder
            .createPath("Node8-Preload-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Preload-Engage", node8PreloadEngage);

    // Node8x3-Engage
    ArrayList<Command> node8x3Engage =
        autoBuilder.createPaths(
            "Node8x3-Engage", kFastPathConstraints, kFastPathConstraints, kEngagePathConstraints);
    AutoChooser.addPathGroup(scorePreload.get(), "Node8x3-Engage", node8x3Engage);

    // Node8-Mobility-Engage
    Command node8MobilityEngage =
        autoBuilder
            .createPath("Node8-Mobility-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node8-Mobility-Engage", node8MobilityEngage);

    // Node2-Preload-Engage
    Command node2PreloadEngage =
        autoBuilder
            .createPath("Node2-Preload-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Preload-Engage", node2PreloadEngage);

    // Node8x2-Ready
    ArrayList<Command> node8x2Ready =
        autoBuilder.createPaths("Node8x2-Ready", kSafePathConstraints);
    AutoChooser.addPathGroup(scorePreload.get(), "Node8x2-Ready", node8x2Ready);

    // Node2x3-Engage
    ArrayList<Command> node2x3Engage =
        autoBuilder.createPaths(
            "Node2x3-Engage", kFastPathConstraints, kFastPathConstraints, kEngagePathConstraints);
    AutoChooser.addPathGroup(scorePreload.get(), "Node2x3-Engage", node2x3Engage);

    // Node2-Engage
    Command node2Engage =
        autoBuilder
            .createPath("Node2-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreload.get());
    AutoChooser.createSinglePath("Node2-Engage", node2Engage);

    AutoChooser.sendChooserToDashboard("Auto Chooser");
  }

  public Command runParallelWithPath(Command command) {
    return new InstantCommand(() -> command.schedule());
  }

  public Command getSelectedPath() {
    Command zeroGyroTeleop = new InstantCommand();
    if (DriverStation.getAlliance() == Alliance.Red) {
      zeroGyroTeleop =
          new InstantCommand(
              () ->
                  swerveSubsystem.setGyroYaw(
                      (swerveSubsystem.getYaw().times(-1).plus(Rotation2d.fromDegrees(180)))
                          .getDegrees()));
    }

    return AutoChooser.getCommand().andThen(zeroGyroTeleop);
  }
}
