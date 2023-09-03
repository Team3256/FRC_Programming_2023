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
import frc.robot.helpers.DebugCommandBase;
import java.util.function.BooleanSupplier;

public class StowEndEffector extends DebugCommandBase {
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private BooleanSupplier isCurrentPieceCone;

  public StowEndEffector(
      Elevator elevatorSubsystem, Arm armSubsystem, BooleanSupplier isCurrentPieceCone) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;

    addRequirements(elevatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    if (isCurrentPieceCone.getAsBoolean()) {
      Commands.sequence(
              new SetEndEffectorState(
                  elevatorSubsystem, armSubsystem, EndEffectorPreset.STOW_CONE, true),
              new ZeroElevator(elevatorSubsystem).asProxy())
          .schedule();
    } else {
      Commands.sequence(
              new SetEndEffectorState(
                  elevatorSubsystem, armSubsystem, EndEffectorPreset.STOW_CUBE, true),
              new ZeroElevator(elevatorSubsystem).asProxy())
          .schedule();
    }
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
