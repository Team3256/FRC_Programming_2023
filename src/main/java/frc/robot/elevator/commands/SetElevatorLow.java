// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import frc.robot.elevator.Elevator;

public class SetElevatorLow extends SetElevatorHeight {
  public SetElevatorLow(Elevator elevatorSubsystem) {
    super(elevatorSubsystem, kElevatorLowPositionMeters);
  }
}
