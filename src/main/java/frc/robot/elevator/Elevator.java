// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.swerve.helpers.Conversions.falconToDistance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static TalonFX elevatorMotor;

  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorID);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    System.out.println("Elevator initialized");
    off();
  }

  public void setSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
    System.out.println("Elevator speed: " + speed);
  }

  public static double getPosition() {
    return falconToDistance(
        elevatorMotor.getSelectedSensorPosition(), elevatorMotorDiameter, elevatorGearRatio);
  }

  public void zeroElevator() {
    elevatorMotor.set(ControlMode.Position, 0);
  }

  public boolean isMotorCurrentSpiking() {
    return elevatorMotor.getSupplyCurrent() >= kElevatorCurrentThreshold;
  }

  public void off() {
    elevatorMotor.neutralOutput();
    System.out.println("Elevator off");
  }
}
