// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.arm.ArmSubsystem;
import frc.robot.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class SetElevatorProfiledPID extends ProfiledPIDCommand {
  DoubleSupplier velocity;
  Elevator elevatorSubsystem;
  boolean movingSetpoint = false;

  public SetElevatorProfiledPID(Elevator elevatorSubsystem) {
    super(
        new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(0, 0)),
        () -> 100,
        () -> 2000,
        (output, setpoint) -> elevatorSubsystem.setPercentSpeed(output),
        elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putData(getController());
    SmartDashboard.putNumber("Velocity Setpoint", 1200);
    SmartDashboard.putNumber("Elevator KFF", ArmConstants.kFF);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPercentSpeed(0);
  }

  @Override
  public void execute() {
    super.execute();
    if (movingSetpoint) {
      SmartDashboard.putNumber("Velocity Setpoint", velocity.getAsDouble());
    }
  }
}