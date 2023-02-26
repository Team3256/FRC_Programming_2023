// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.elevator.Elevator;

public class SetElevatorHeight extends ProfiledPIDCommand {
  public SetElevatorHeight(Elevator elevatorSubsystem, double setpointPositionMeters) {
    super(
        new ProfiledPIDController(kP, kI, kD, kElevatorContraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPositionMeters,
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }

  public SetElevatorHeight(Elevator elevatorSubsystem, Elevator.ElevatorPosition elevatorPosition) {
    this(elevatorSubsystem, elevatorPosition.position);
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber(
        "Elevator setpoint position", Units.metersToInches(getController().getSetpoint().position));
  }
}
