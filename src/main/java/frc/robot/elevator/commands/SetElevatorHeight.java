// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.elevator.Elevator;

public class SetElevatorHeight extends ProfiledPIDCommand {

  /**
   * Constructor for setting elevator height for the Low, Mid, High levels
   *
   * @param elevatorSubsystem
   * @param setpointPositionMeters
   */
  public SetElevatorHeight(
      Elevator elevatorSubsystem, Elevator.ElevatorPosition setpointPositionMeters) {
    super(
        new ProfiledPIDController(
            Preferences.getDouble(ElevatorPreferencesKeys.kPKey, kP),
            Preferences.getDouble(ElevatorPreferencesKeys.kIKey, kI),
            Preferences.getDouble(ElevatorPreferencesKeys.kDKey, kD),
            kElevatorContraints),
        elevatorSubsystem::getElevatorPosition,
        elevatorSubsystem.getPreferencesSetpoint(setpointPositionMeters),
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }

  /**
   * Constructor for setting the elevator to a setpoint in the parameters
   *
   * @param elevatorSubsystem
   * @param setpointPositionMeters
   */
  public SetElevatorHeight(Elevator elevatorSubsystem, double setpointPositionMeters) {
    super(
        new ProfiledPIDController(
            Preferences.getDouble(ElevatorPreferencesKeys.kPKey, kP),
            Preferences.getDouble(ElevatorPreferencesKeys.kIKey, kI),
            Preferences.getDouble(ElevatorPreferencesKeys.kDKey, kD),
            kElevatorContraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPositionMeters,
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }
}
