// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorStartingPositionMeters;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerve;
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

  public void pitRoutine() {
    Command startRoutine = new WaitCommand(1).until(driver.a());
    Command tests = new WaitCommand(1).beforeStarting(startRoutine);
    if (kIntakeEnabled) {
      tests.andThen(intakeCommands());
    }
    if (kArmEnabled) {
      tests.andThen(armCommands());
    }
    if (kSwerveEnabled) {
      tests.andThen(swerveCommands());
    }
    if (kElevatorEnabled) {
      tests.andThen(elevatorCommands());
    }
    tests.schedule();
  }

  private Command elevatorCommands() {
    Command zeroElevator = new ZeroElevator(elevatorSubsystem).until(driver.a());
    Command setElevatorHeightHIGH = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH)
        .until(driver.a());
    Command setElevatorHeightMID = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID)
        .until(driver.a());
    Command setElevatorHeightLOW = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW)
        .until(driver.a());
    Command setElevatorToStart = new SetElevatorHeight(elevatorSubsystem, kElevatorStartingPositionMeters)
        .until(driver.a());

    return zeroElevator.andThen(
        setElevatorHeightHIGH.andThen(
            setElevatorHeightMID.andThen(setElevatorHeightLOW.andThen(setElevatorToStart))));
  }

  public Command intakeCommands() {
    Command intakeCone = new IntakeCone(intakeSubsystem).until(driver.a());
    Command outtakeCone = new IntakeCube(intakeSubsystem).until(driver.a());
    Command intakeCube = new IntakeCube(intakeSubsystem).until(driver.a());
    Command outtakeCube = new IntakeCone(intakeSubsystem).until(driver.a());

    return intakeCone.andThen(outtakeCone.andThen(intakeCube.andThen(outtakeCube)));
  }

  public Command armCommands() {
    Command setArmAngleHorizontal = new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(-12)).until(driver.a());
    Command setArmAngleHalfway = new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(45)).until(driver.a());
    Command setArmAngleVertical = new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(90)).until(driver.a());

    return setArmAngleHorizontal.andThen(setArmAngleHalfway.andThen(setArmAngleVertical));
  }

  public Command swerveCommands() {
    Command lockSwerve = new LockSwerve(swerveSubsystem).until(driver.a());
    Command teleopSwerveForward = // move forward
        new TeleopSwerve(
            swerveSubsystem,
            () -> kSwervePitTestSpeed,
            () -> 0,
            () -> 0,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());

    Command telopSwerveBackward = // move backward
        new TeleopSwerve(
            swerveSubsystem,
            () -> -kSwervePitTestSpeed,
            () -> 0,
            () -> 0,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());

    Command teleopSwerveRight = // move right
        new TeleopSwerve(
            swerveSubsystem,
            () -> 0,
            () -> kSwervePitTestSpeed,
            () -> 0,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());

    Command teleopSwerveLeft = // move left
        new TeleopSwerve(
            swerveSubsystem,
            () -> 0,
            () -> -kSwervePitTestSpeed,
            () -> 0,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());

    Command teleopSwerveRotateRight = // rotate right
        new TeleopSwerve(
            swerveSubsystem,
            () -> 0,
            () -> 0,
            () -> kSwervePitTestSpeed,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());

    Command teleopSwerveRotateLeft = // rotate left
        new TeleopSwerve(
            swerveSubsystem,
            () -> 0,
            () -> 0,
            () -> -kSwervePitTestSpeed,
            kFieldRelative,
            kOpenLoop)
            .until(driver.a());
    return lockSwerve.andThen(
        teleopSwerveForward.andThen(
            telopSwerveBackward.andThen(
                teleopSwerveRight.andThen(
                    teleopSwerveLeft.andThen(
                        teleopSwerveRotateRight.andThen(teleopSwerveRotateLeft))))));
  }
}
