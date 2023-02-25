// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.helpers.WaitCommand;

public class SetArmElevatorStart {
  public static Command getCommand(Elevator elevatorSubsystem, Arm armSubsystem) {
    return new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(180))
        .andThen(
            new ParallelCommandGroup(
                new WaitCommand(0.6)
                    .andThen(
                        new ProxyCommand(
                            new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(80)))),
                new ZeroElevator(elevatorSubsystem)));
  }
}
