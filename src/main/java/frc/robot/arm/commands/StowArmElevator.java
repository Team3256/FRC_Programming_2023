// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;

public class StowArmElevator extends ParallelCommandGroup {
  public StowArmElevator(Elevator elevatorSubsystem, Arm armSubsystem) {
    addCommands(
        new WaitCommand(0.17).andThen(new ZeroElevator(elevatorSubsystem)),
        new SetArmAngle(armSubsystem, Arm.ArmPreset.DEFAULT));
  }

  public StowArmElevator(
      Elevator elevatorSubsystem, Arm armSubsystem, double armWaitTime, double elevatorWaitTime) {
    addCommands(
        new WaitCommand(elevatorWaitTime).andThen(new ZeroElevator(elevatorSubsystem)),
        new WaitCommand(armWaitTime).andThen(new SetArmAngle(armSubsystem, Arm.ArmPreset.DEFAULT)));
  }
}
