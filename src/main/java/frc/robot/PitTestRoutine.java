// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorStartingPosition;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorExtension;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerveX;
import frc.robot.swerve.commands.TeleopSwerve;

public class PitTestRoutine {
  Elevator elevatorSubsystem;
  Intake intakeSubsystem;
  SwerveDrive swerveSubsystem;
  Arm armSubsystem;
  private final CommandXboxController driver = new CommandXboxController(0);

  public PitTestRoutine(
      Elevator elevatorSubsystem,
      Intake intakeSubsystem,
      SwerveDrive swerveSubsystem,
      Arm armSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public void runPitRoutine() {
    Command startRoutine = new WaitCommand(1).until(driver.a());
    Command tests = new WaitCommand(1).beforeStarting(startRoutine);

    Command intakeTests = new InstantCommand();
    Command armTests = new InstantCommand();
    Command elevatorTests = new InstantCommand();
    Command swerveTests = new InstantCommand();

    if (kIntakeEnabled) {
      intakeTests = intakeCommands();
    }
    if (kArmEnabled) {
      armTests = armCommands();
    }
    if (kSwerveEnabled) {
      swerveTests = swerveCommands();
    }
    if (kElevatorEnabled) {
      elevatorTests = elevatorCommands();
    }
    tests
        .andThen(intakeTests)
        .andThen(armTests)
        .andThen(swerveTests)
        .andThen(elevatorTests)
        .schedule();
  }

  private Command elevatorCommands() {
    Command zeroElevator = new ZeroElevator(elevatorSubsystem).until(driver.b());
    Command setElevatorHeightHIGH =
        new SetElevatorExtension(elevatorSubsystem, Elevator.ElevatorPreset.CUBE_HIGH)
            .until(driver.b());
    Command setElevatorHeightMID =
        new SetElevatorExtension(elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_MID)
            .until(driver.b());
    Command setElevatorHeightLOW =
        new SetElevatorExtension(elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK)
            .until(driver.b());
    Command setElevatorToStart =
        new SetElevatorExtension(elevatorSubsystem, kElevatorStartingPosition).until(driver.b());

    return new SequentialCommandGroup(
        zeroElevator,
        setElevatorHeightHIGH,
        setElevatorHeightMID,
        setElevatorHeightLOW,
        zeroElevator,
        setElevatorToStart);
  }

  public Command intakeCommands() {
    Command intakeCone = new IntakeCone(intakeSubsystem).until(driver.b());
    Command outtakeCone = new IntakeCube(intakeSubsystem).until(driver.b());
    Command intakeCube = new IntakeCube(intakeSubsystem).until(driver.b());
    Command outtakeCube = new IntakeCone(intakeSubsystem).until(driver.b());

    return new SequentialCommandGroup(intakeCone, outtakeCone, intakeCube, outtakeCube);
  }

  public Command armCommands() {
    Command setArmAngleHorizontal =
        new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(0)).until(driver.b());
    Command setArmAngleHalfway =
        new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(45)).until(driver.b());
    Command setArmAngleVertical =
        new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(90)).until(driver.b());

    return new SequentialCommandGroup(
        setArmAngleHorizontal, setArmAngleHalfway, setArmAngleVertical);
  }

  public Command swerveCommands() {
    Command lockSwerve = new LockSwerveX(swerveSubsystem).until(driver.b());
    Command teleopSwerveForward = // move forward
        new TeleopSwerve(
                swerveSubsystem,
                () -> kSwervePitTestSpeed,
                () -> 0,
                () -> 0,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());

    Command teleopSwerveBackward = // move backward
        new TeleopSwerve(
                swerveSubsystem,
                () -> -kSwervePitTestSpeed,
                () -> 0,
                () -> 0,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());

    Command teleopSwerveRight = // move right
        new TeleopSwerve(
                swerveSubsystem,
                () -> 0,
                () -> kSwervePitTestSpeed,
                () -> 0,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());

    Command teleopSwerveLeft = // move left
        new TeleopSwerve(
                swerveSubsystem,
                () -> 0,
                () -> -kSwervePitTestSpeed,
                () -> 0,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());

    Command teleopSwerveRotateRight = // rotate right
        new TeleopSwerve(
                swerveSubsystem,
                () -> 0,
                () -> 0,
                () -> kSwervePitTestSpeed,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());

    Command teleopSwerveRotateLeft = // rotate left
        new TeleopSwerve(
                swerveSubsystem,
                () -> 0,
                () -> 0,
                () -> -kSwervePitTestSpeed,
                kFieldRelative,
                kOpenLoop)
            .until(driver.b());
    return new SequentialCommandGroup(
        lockSwerve,
        teleopSwerveForward,
        teleopSwerveBackward,
        teleopSwerveRight,
        teleopSwerveLeft,
        teleopSwerveRotateRight,
        teleopSwerveRotateLeft);
  }
}
