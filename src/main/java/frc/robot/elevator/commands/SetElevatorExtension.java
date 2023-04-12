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
import frc.robot.elevator.Elevator.ElevatorPreset;

public class SetElevatorExtension extends ProfiledPIDCommand {
  private double setpointPosition;
  private Elevator elevatorSubsystem;
  private ElevatorPreset elevatorPreset;

  /**
   * Constructor for setting the elevator to a setpoint in the parameters
   *
   * @param elevatorSubsystem
   * @param setpointPosition
   */
  public SetElevatorExtension(Elevator elevatorSubsystem, double setpointPosition) {
    super(
        new ProfiledPIDController(
            Preferences.getDouble(ElevatorPreferencesKeys.kPKey, kElevatorP),
            Preferences.getDouble(ElevatorPreferencesKeys.kIKey, kElevatorI),
            Preferences.getDouble(ElevatorPreferencesKeys.kDKey, kElevatorD),
            kElevatorConstraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPosition,
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    this.setpointPosition = setpointPosition;
    this.elevatorSubsystem = elevatorSubsystem;

    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }

  /**
   * Constructor for setting elevator height for the levels specified in the elevator preferences
   * hash map
   *
   * @param elevatorSubsystem
   * @param elevatorPreset
   */
  public SetElevatorExtension(Elevator elevatorSubsystem, ElevatorPreset elevatorPreset) {
    this(elevatorSubsystem, elevatorSubsystem.getElevatorSetpoint(elevatorPreset));
    this.elevatorPreset = elevatorPreset;
  }

  @Override
  public void initialize() {
    super.initialize();

    // update at runtime in case robot prefs changed
    if (elevatorPreset != null) {
      setpointPosition = elevatorSubsystem.getElevatorSetpoint(elevatorPreset);
      getController().setGoal(setpointPosition);
    }

    System.out.println(
        this.getName()
            + " started (preset: "
            + this.elevatorPreset
            + ", height: "
            + setpointPosition
            + " meters)");
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        this.getName()
            + " finished (preset: "
            + this.elevatorPreset
            + ", height: "
            + setpointPosition
            + " meters)");
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
