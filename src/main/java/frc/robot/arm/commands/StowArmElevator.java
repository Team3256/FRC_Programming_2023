// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorExtension;
import java.util.function.BooleanSupplier;

public class StowArmElevator extends ParallelCommandGroup {
  public StowArmElevator(
      Elevator elevatorSubsystem, Arm armSubsystem, BooleanSupplier isCurrentPieceCone) {

    addCommands(
        new ConditionalCommand(
            Commands.parallel(
                new WaitCommand(0.17)
                    .andThen(
                        new SetElevatorExtension(
                            elevatorSubsystem, Elevator.ElevatorPreset.STOW_CONE)),
                new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CONE)),
            Commands.parallel(
                new WaitCommand(0.17)
                    .andThen(
                        new SetElevatorExtension(
                            elevatorSubsystem, Elevator.ElevatorPreset.STOW_CUBE)),
                new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CUBE)),
            isCurrentPieceCone));
  }

  public StowArmElevator(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      double armWaitTime,
      double elevatorWaitTime,
      BooleanSupplier isCurrentPieceCone) {
    addCommands(
        new ConditionalCommand(
            Commands.parallel(
                new WaitCommand(elevatorWaitTime)
                    .andThen(
                        new SetElevatorExtension(
                            elevatorSubsystem, Elevator.ElevatorPreset.STOW_CONE)),
                new WaitCommand(armWaitTime)
                    .andThen(new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CONE))),
            Commands.parallel(
                new WaitCommand(elevatorWaitTime)
                    .andThen(
                        new SetElevatorExtension(
                            elevatorSubsystem, Elevator.ElevatorPreset.STOW_CUBE)),
                new WaitCommand(armWaitTime)
                    .andThen(new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW_CUBE))),
            isCurrentPieceCone));
  }
}
