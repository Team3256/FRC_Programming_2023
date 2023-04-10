// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import java.util.function.BooleanSupplier;

public class StowEndEffector extends ParallelCommandGroup {
  public StowEndEffector(
      Elevator elevatorSubsystem, Arm armSubsystem, BooleanSupplier isCurrentPieceCone) {
    addCommands(
        new ConditionalCommand(
            new SetEndEffectorState(
                elevatorSubsystem, armSubsystem, SetEndEffectorState.EndEffectorPreset.STOW_CONE),
            new SetEndEffectorState(
                elevatorSubsystem, armSubsystem, SetEndEffectorState.EndEffectorPreset.STOW_CUBE),
            isCurrentPieceCone));
  }
}
