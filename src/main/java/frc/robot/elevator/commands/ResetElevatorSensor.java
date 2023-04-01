// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class ResetElevatorSensor extends CommandBase {
  Elevator elevatorSubsystem;

  public ResetElevatorSensor(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setInputVoltage(kDownSpeedVolts);
    System.out.println("Zeroing elevator");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Zero elevator finished");
    elevatorSubsystem.off();
    if (!interrupted) elevatorSubsystem.zeroElevator();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isMotorCurrentSpiking();
  }
}
