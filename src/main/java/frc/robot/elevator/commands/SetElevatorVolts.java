// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.elevator.Elevator;

public class SetElevatorVolts extends CommandBase {
  private Elevator elevatorSubsystem;
  private double volts = 0;

  public SetElevatorVolts(Elevator elevatorSubsystem, double volts) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.volts = volts;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setInputVoltage(volts);
    if (Constants.kDebugEnabled) {
      System.out.println(this.getName() + " started (volts: " + volts + ")");
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
