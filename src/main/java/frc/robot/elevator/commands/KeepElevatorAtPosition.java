// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class KeepElevatorAtPosition extends CommandBase {
  private Elevator elevatorSubsystem;

  public KeepElevatorAtPosition(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    new SetElevatorExtension(elevatorSubsystem, elevatorSubsystem.getElevatorPosition())
        .repeatedly()
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
