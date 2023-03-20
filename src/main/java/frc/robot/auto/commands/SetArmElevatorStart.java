// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;

public class SetArmElevatorStart extends ParallelCommandGroup {
  public SetArmElevatorStart(Elevator elevatorSubsystem, Arm armSubsystem) {
    // TODO arm not going to default
    // TODO remove timings
    addCommands(
        new ParallelDeadlineGroup(
                new WaitCommand(0.45), new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(135)))
            .andThen(new SetArmAngle(armSubsystem, ArmPosition.DEFAULT)),
        new WaitCommand(0.3).andThen(new ZeroElevator(elevatorSubsystem)));
  }
}
