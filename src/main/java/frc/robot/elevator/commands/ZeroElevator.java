// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import frc.robot.elevator.Elevator;
import frc.robot.helpers.DebugCommandBase;

public class ZeroElevator extends DebugCommandBase {
  Elevator elevatorSubsystem;

  public ZeroElevator(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    elevatorSubsystem.setInputVoltage(kDownSpeedVolts);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevatorSubsystem.off();
    if (!interrupted) elevatorSubsystem.zeroElevator();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isMotorCurrentSpiking();
  }
}
