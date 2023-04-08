// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import frc.robot.elevator.Elevator;
import frc.robot.helpers.DebugCommandBase;

public class SetElevatorVolts extends DebugCommandBase {
  private Elevator elevatorSubsystem;
  private double volts = 0;

  public SetElevatorVolts(Elevator elevatorSubsystem, double volts) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.volts = volts;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    elevatorSubsystem.setInputVoltage(volts);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevatorSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
