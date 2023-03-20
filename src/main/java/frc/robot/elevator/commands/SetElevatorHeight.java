// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPreset;

public class SetElevatorHeight extends ProfiledPIDCommand {
  private double setpointPositionMeters;
  private ElevatorPreset elevatorPreset;

  public SetElevatorHeight(Elevator elevatorSubsystem, double setpointPositionMeters) {
    super(
        new ProfiledPIDController(kP, kI, kD, kElevatorContraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPositionMeters,
        (output, setpoint) -> elevatorSubsystem.setInputVoltage(
            output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    this.setpointPositionMeters = setpointPositionMeters;
    getController().setTolerance(kTolerancePosition, kToleranceVelocity);
    addRequirements(elevatorSubsystem);
  }

  public SetElevatorHeight(Elevator elevatorSubsystem, ElevatorPreset elevatorPreset) {
    this(elevatorSubsystem, elevatorPreset.position);
    this.elevatorPreset = elevatorPreset;
  }

  @Override
  public void initialize() {
    super.initialize();
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " started (preset: "
              + this.elevatorPreset
              + ", height: "
              + setpointPositionMeters
              + " meters)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " ended (preset: "
              + this.elevatorPreset
              + ", height: "
              + setpointPositionMeters
              + " meters)");
    }
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
