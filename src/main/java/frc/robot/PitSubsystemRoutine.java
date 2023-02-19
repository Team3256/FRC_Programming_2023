// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorStartingPose;
import static frc.robot.swerve.SwerveConstants.kFieldRelative;
import static frc.robot.swerve.SwerveConstants.kOpenLoop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerve;
import frc.robot.swerve.commands.TeleopSwerve;

public class PitSubsystemRoutine {
  Elevator elevatorSubsystem;
  Intake intakeSubsystem;
  SwerveDrive swerveSubsystem;
  private final CommandXboxController driver = new CommandXboxController(0);

  public PitSubsystemRoutine(Elevator elevator, Intake intake, SwerveDrive swerve) {
    elevatorSubsystem = elevator;
    intakeSubsystem = intake;
    swerveSubsystem = swerve;
  }

  public void pitRoutine() {
    Command startRoutine = new WaitCommand(1).until(driver.a());
    Command tests = new WaitCommand(1).beforeStarting(startRoutine);
    if (kIntakeEnabled) {
      tests.andThen(intakeCommands());
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
    Command setElevatorHeightHIGH = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH).until(driver.a());
    Command setElevatorHeightMID = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID).until(driver.a());
    Command setElevatorHeightLOW = new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW).until(driver.a());
    Command setElevatorHeight = new SetElevatorHeight(elevatorSubsystem, kElevatorStartingPose).until(driver.a());

    return zeroElevator.andThen(setElevatorHeightHIGH.andThen(setElevatorHeightMID.andThen
            (setElevatorHeightLOW.andThen(setElevatorHeight))));
  }

  public Command intakeCommands() {
    Command intakeCone = new IntakeCone(intakeSubsystem).until(driver.a());
    Command intakeCube = new IntakeCube(intakeSubsystem).until(driver.a());

    return intakeCube.andThen(intakeCone);
  }

  public Command swerveCommands() {
    Command lockSwerve = new LockSwerve(swerveSubsystem).until(driver.a());
    Command teleopSwerveForward = //move forward
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> 0.3,
                    () -> 0,
                    () -> 0,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());

    Command telopSwerveBackward = //move backward
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> -0.3,
                    () -> 0,
                    () -> 0,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());

    Command teleopSwerveRight = //move right
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> 0,
                    () -> 0.3,
                    () -> 0,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());

    Command teleopSwerveLeft = //move left
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> 0,
                    () -> -0.3,
                    () -> 0,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());

    Command teleopSwerveRotateRight = //rotate right
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> 0,
                    () -> 0,
                    () -> 0.3,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());

    Command teleopSwerveRotateLeft = //rotate left
            new TeleopSwerve(
                    swerveSubsystem,
                    () -> 0,
                    () -> 0,
                    () -> -0.3,
                    kFieldRelative,
                    kOpenLoop).until(driver.a());
    return lockSwerve.andThen(teleopSwerveForward.andThen(telopSwerveBackward.andThen
            (teleopSwerveRight.andThen(teleopSwerveLeft.andThen
                    (teleopSwerveRotateRight.andThen(teleopSwerveRotateLeft))))));
  }
}
