// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;
import java.util.function.BooleanSupplier;

public class StowArmElevator extends ParallelCommandGroup {
  public StowArmElevator(
      Elevator elevatorSubsystem, Arm armSubsystem, BooleanSupplier isCurrentPieceCone) {

    addCommands(
        new WaitCommand(0.17).andThen(new ZeroElevator(elevatorSubsystem)),
        new ConditionalCommand(
            new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CONE),
            new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CUBE),
            isCurrentPieceCone));
  }

  public StowArmElevator(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      double armWaitTime,
      double elevatorWaitTime,
      BooleanSupplier isCurrentPieceCone) {
    addCommands(
        new WaitCommand(elevatorWaitTime).andThen(new ZeroElevator(elevatorSubsystem)),
        new WaitCommand(armWaitTime)
            .andThen(
                new ConditionalCommand(
                    new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CONE),
                    new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CUBE),
                    isCurrentPieceCone)));
  }
}
