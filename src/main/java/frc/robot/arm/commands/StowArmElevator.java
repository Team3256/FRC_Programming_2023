// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;

public class StowArmElevator extends ParallelCommandGroup {
  public StowArmElevator(Elevator elevatorSubsystem, Arm armSubsystem) {
    super.addCommands(
        new ZeroElevator(elevatorSubsystem),
        new SetArmAngle(armSubsystem, Arm.ArmPosition.DEFAULT));
  }
}