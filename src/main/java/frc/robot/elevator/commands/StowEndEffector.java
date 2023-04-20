// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState.EndEffectorPreset;
import frc.robot.helpers.ParentCommand;
import java.util.function.BooleanSupplier;

public class StowEndEffector extends ParentCommand {
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private BooleanSupplier isCurrentPieceCone;

  public StowEndEffector(
      Elevator elevatorSubsystem, Arm armSubsystem, BooleanSupplier isCurrentPieceCone) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    if (isCurrentPieceCone.getAsBoolean()) {
      addChildCommands(
          Commands.sequence(
              new SetEndEffectorState(elevatorSubsystem, armSubsystem, EndEffectorPreset.STOW_CONE),
              // new SetArmAngle(armSubsystem, ArmPreset.STOW_CONE).asProxy(),
              // new ScheduleCommand(new KeepArm(armSubsystem)).asProxy(),
              // new SetElevatorExtension(elevatorSubsystem,
              // ElevatorPreset.STOW_CONE).asProxy(),
              new ZeroElevator(elevatorSubsystem).asProxy()));
    } else {
      addChildCommands(
          Commands.sequence(
              new SetEndEffectorState(elevatorSubsystem, armSubsystem, EndEffectorPreset.STOW_CUBE),
              // new SetArmAngle(armSubsystem, ArmPreset.STOW_CUBE).asProxy(),
              // new ScheduleCommand(new KeepArm(armSubsystem)).asProxy(),
              // new SetElevatorExtension(elevatorSubsystem,
              // ElevatorPreset.STOW_CUBE).asProxy(),
              new ZeroElevator(elevatorSubsystem).asProxy()));
    }
    super.initialize();
  }
}
