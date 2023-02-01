// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.elevator.Elevator;

public class ElevatorSetHeight extends PIDCommand {
  Elevator elevatorSubsystem;

  public ElevatorSetHeight(Elevator elevatorSubsystem, double target) {
    super(
        new PIDController(kP, kI, kD),
        Elevator::getPosition,
        target,
        elevatorSubsystem::setSpeed,
        elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
    getController().setTolerance(kTolerancePosition, kToleranceRate);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.off();
  }
}
