// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.elevator.Elevator;

public class ElevatorSetHeight extends ProfiledPIDCommand {
  private Elevator elevatorSubsystem;

  public ElevatorSetHeight(Elevator elevatorSubsystem, double setpointPositionMeters) {
    super(
        new ProfiledPIDController(kP, kI, kD, kElevatorContraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPositionMeters,
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    this.elevatorSubsystem = elevatorSubsystem;

    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    super.execute();
    System.out.println("Height: " + elevatorSubsystem.getElevatorPosition());
  }
}
