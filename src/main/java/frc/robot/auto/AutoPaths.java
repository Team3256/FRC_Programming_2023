// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.AutoConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
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
import frc.robot.swerve.commands.AutoBalance;
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

    Supplier<Command> scorePreloadCone = () -> new InstantCommand();
    Supplier<Command> scorePreloadCube = () -> new InstantCommand();

    if (swerveSubsystem != null
        && intakeSubsystem != null
        && armSubsystem != null
        && elevatorSubsystem != null) {

      autoEventMap.put(
          "defaultPosition",
          () ->
              runParallelWithPath(
                  Commands.parallel(
                          new StowArmElevator(elevatorSubsystem, armSubsystem),
                          new IntakeOff(intakeSubsystem))
                      .asProxy()
                      .withName("defaultPosition")));

      autoEventMap.put(
          "intakeCone",
          () ->
              runParallelWithPath(
                      Commands.deadline(
                          new IntakeCone(intakeSubsystem),
                          new SetElevatorHeight(
                              elevatorSubsystem, Elevator.ElevatorPreset.GROUND_INTAKE),
                          new SetArmAngle(armSubsystem, ArmPreset.GROUND_INTAKE)))
                  .asProxy()
                  .withName("intakeCone"));

      autoEventMap.put(
          "intakeCube",
          () ->
              runParallelWithPath(
                      Commands.deadline(
                          new IntakeCube(intakeSubsystem),
                          new SetElevatorHeight(
                              elevatorSubsystem, Elevator.ElevatorPreset.GROUND_INTAKE),
                          new SetArmAngle(armSubsystem, ArmPreset.GROUND_INTAKE)))
                  .asProxy()
                  .withName("intakeCube"));

      autoEventMap.put(
          "cubeHigh",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPreset.CUBE_HIGH),
                      new SetArmAngle(armSubsystem, ArmPreset.CUBE_HIGH))
                  .withTimeout(2.25)
                  .andThen(new IntakeCone(intakeSubsystem))
                  .asProxy()
                  .withName("cubeHigh"));

      autoEventMap.put(
          "coneHigh",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPreset.CONE_HIGH),
                      new SetArmAngle(armSubsystem, ArmPreset.CONE_HIGH))
                  .withTimeout(2.25)
                  .andThen(new IntakeCube(intakeSubsystem))
                  .asProxy()
                  .withName("coneHigh"));

      autoEventMap.put(
          "cubeMid",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_MID),
                      new SetArmAngle(armSubsystem, ArmPreset.CUBE_MID))
                  .withTimeout(2.25)
                  .andThen(new IntakeCone(intakeSubsystem))
                  .asProxy()
                  .withName("cubeMid"));

      autoEventMap.put(
          "coneMid",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_MID),
                      new SetArmAngle(armSubsystem, ArmPreset.CONE_MID))
                  .withTimeout(2.25)
                  .andThen(new IntakeCube(intakeSubsystem))
                  .asProxy()
                  .withName("coneMid"));

      autoEventMap.put(
          "cubeLow",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_LOW),
                      new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW))
                  .withTimeout(2.25)
                  .andThen(new IntakeCone(intakeSubsystem))
                  .asProxy()
                  .withName("cubeLow"));

      autoEventMap.put(
          "coneLow",
          () ->
              Commands.parallel(
                      new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_LOW),
                      new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW))
                  .withTimeout(2.25)
                  .andThen(new IntakeCube(intakeSubsystem))
                  .asProxy()
                  .withName("coneLow"));

      autoEventMap.put(
          "engage",
          () ->
              new AutoBalance(swerveSubsystem)
                  .andThen(new LockSwerveX(swerveSubsystem))
                  .asProxy()
                  .withName("engage"));

      scorePreloadCube =
          () ->
              Commands.parallel(
                      new ZeroElevator(elevatorSubsystem),
                      new SetArmAngle(armSubsystem, ArmPreset.CUBE_HIGH))
                  .withTimeout(2.25)
                  .andThen(new IntakeCone(intakeSubsystem).withTimeout(1))
                  .asProxy()
                  .withName("scorePreloadCube");

      scorePreloadCone =
          () ->
              Commands.parallel(
                      new ZeroElevator(elevatorSubsystem),
                      new SetArmAngle(armSubsystem, ArmPreset.CONE_HIGH))
                  .withTimeout(2.25)
                  .andThen(new IntakeCone(intakeSubsystem).withTimeout(1))
                  .asProxy()
                  .withName("scorePreloadCone");
    }

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem, autoEventMap);

    // Test - Do not select
    // Command test = autoBuilder
    // .createPath("Test - Do not select", kEngagePathConstraints, true);
    // AutoChooser.createSinglePath("Test - Do not select", test);

    Command node8Mobility =
        autoBuilder
            .createPath("Node8-Mobility", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node8-Mobility", node8Mobility);

    Command node2Mobility =
        autoBuilder
            .createPath("Node2-Mobility", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node2-Mobility", node2Mobility);

    // Node5-Engage
    ArrayList<Command> node5Engage =
        autoBuilder.createPaths("Node5-Engage", kEngagePathConstraints);
    AutoChooser.addPathGroup(
        scorePreloadCube.get(), "Node5-Engage", node5Engage, new LockSwerveX(swerveSubsystem));

    // Node8-Preload-Ready
    Command node8PreloadReady =
        autoBuilder
            .createPath("Node8-Preload-Ready", kSafePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node8-Preload-Ready", node8PreloadReady);

    // Node5-Mobility-Engage
    Command node5MobilityEngage =
        autoBuilder
            .createPath("Node5-Mobility-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node5-Mobility-Engage", node5MobilityEngage);

    // Node8x2
    Command node9x2 =
        autoBuilder
            .createPath("Node9x2", kFastPathConstraints, true)
            .beforeStarting(scorePreloadCone.get().asProxy());
    AutoChooser.createSinglePath("Node9x2", node9x2);

    // Node2x2-Engage
    ArrayList<Command> node2x2Engage =
        autoBuilder.createPaths("Node2x2-Engage", kSafePathConstraints, kEngagePathConstraints);
    AutoChooser.addPathGroup(
        scorePreloadCube.get(), "Node2x2-Engage", node2x2Engage, new LockSwerveX(swerveSubsystem));

    // Node2-Preload-Ready
    Command node2PreloadReady =
        autoBuilder
            .createPath("Node2-Preload-Ready", kSafePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node2-Preload-Ready", node2PreloadReady);

    // Node8-Preload-Engage
    Command node8PreloadEngage =
        autoBuilder
            .createPath("Node8-Preload-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node8-Preload-Engage", node8PreloadEngage);

    // Node8x3-Engage
    ArrayList<Command> node9x2Engage =
        autoBuilder.createPaths(
            "Node9x2+1-Engage",
            kSafePathConstraints,
            kGroundIntakeConstraints,
            kSafePathConstraints,
            kGroundIntakeConstraints,
            kEngagePathConstraints);
    AutoChooser.addPathGroup(scorePreloadCone.get(), "Node9x2+1-Engage", node9x2Engage);

    // Node8-Mobility-Engage
    Command node8MobilityEngage =
        autoBuilder
            .createPath("Node8-Mobility-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node8-Mobility-Engage", node8MobilityEngage);

    // Node2-Preload-Engage
    Command node2PreloadEngage =
        autoBuilder
            .createPath("Node2-Preload-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node2-Preload-Engage", node2PreloadEngage);

    // Node8x2-Ready
    ArrayList<Command> node8x2Ready =
        autoBuilder.createPaths("Node8x2-Ready", kSafePathConstraints);
    AutoChooser.addPathGroup(scorePreloadCube.get(), "Node8x2-Ready", node8x2Ready);

    // Node2-Engage
    Command node2Engage =
        autoBuilder
            .createPath("Node2-Engage", kEngagePathConstraints, true)
            .beforeStarting(scorePreloadCube.get());
    AutoChooser.createSinglePath("Node2-Engage", node2Engage);

    AutoChooser.sendChooserToDashboard("Auto Chooser");
  }

  public Command runParallelWithPath(Command command) {
    return new InstantCommand(() -> command.schedule());
  }

  public Command getSelectedPath() {
    Command zeroGyroTeleop =
        new InstantCommand(
            () ->
                swerveSubsystem.setGyroYaw(
                    (swerveSubsystem.getYaw().times(-1).plus(Rotation2d.fromDegrees(180)))
                        .getDegrees()));
    return AutoChooser.getCommand().finallyDo((interrupted) -> zeroGyroTeleop.schedule());
  }
}
