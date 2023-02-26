// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.helpers.WaitCommand;

public class SetArmElevatorStart extends SequentialCommandGroup {
  public SetArmElevatorStart(Elevator elevatorSubsystem, Arm armSubsystem) {
    addCommands(
        (new ParallelDeadlineGroup(
            new WaitCommand(1),
            new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(135)),
            new ZeroElevator(elevatorSubsystem),
            new SetArmAngle(armSubsystem, Arm.ArmPosition.DEFAULT))));
  }
}
