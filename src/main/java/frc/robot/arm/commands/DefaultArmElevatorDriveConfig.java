// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.kDefaultArmAngle;
import static frc.robot.elevator.ElevatorConstants.kMinHeight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;

public class DefaultArmElevatorDriveConfig extends ParallelCommandGroup {
  public DefaultArmElevatorDriveConfig(Elevator elevatorSubsystem, Arm armSubsystem) {
    addCommands(
        new SetElevatorHeight(elevatorSubsystem, kMinHeight),
        // TODO: fix this number (the 90)
        new SetArmAngle(armSubsystem, new Rotation2d(Units.degreesToRadians(90))));
    new SetArmAngle(armSubsystem, kDefaultArmAngle);
  }
}
