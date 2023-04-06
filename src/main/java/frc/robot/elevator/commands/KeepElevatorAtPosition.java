// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import frc.robot.elevator.Elevator;
import frc.robot.helpers.DebugCommandBase;

public class KeepElevatorAtPosition extends DebugCommandBase {
  private Elevator elevatorSubsystem;

  public KeepElevatorAtPosition(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    new SetElevatorHeight(elevatorSubsystem, elevatorSubsystem.getElevatorPosition())
        .repeatedly()
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
