// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX masterElevatorMotor;
  // private final AdaptiveSlewRateLimiter elevatorRateLimiter = new AdaptiveSlewRateLimiter()
  // private final TalonFX followerElevatorMotor;

  public Elevator() {
    masterElevatorMotor = new TalonFX(ElevatorConstants.MASTER_ELEVATOR_MOTOR_ID);
    //		followerElevatorMotor = new TalonFX(FOLLOWER_ELEVATOR_MOTOR_ID);
    //		followerElevatorMotor.follow(masterElevatorMotor);
    masterElevatorMotor.setNeutralMode(NeutralMode.Brake);
    System.out.println("Elevator initialized");
    off();
  }

  public void setSpeed(double speed) {
    masterElevatorMotor.set(ControlMode.PercentOutput, speed);
    System.out.println("Elevator speed: " + speed);
  }

  public double getPosition() {
    return masterElevatorMotor.getSelectedSensorPosition();
  }

  public void off() {
    masterElevatorMotor.neutralOutput();
    System.out.println("Elevator off");
  }

  @Override
  public void periodic() {
    for (var rateLimit : SwerveConstants.X_RATE_LIMIT_DICT.entrySet()) {
      if (Math.abs(getPosition() - rateLimit.getKey()) < SwerveConstants.POSITION_DIST_DELTA) {
        // Update:
        SwerveConstants.X_ACCEL_RATE_LIMIT = rateLimit.getValue()[0];
        SwerveConstants.X_DECEL_RATE_LIMIT = rateLimit.getValue()[1];
      }
    }

    for (var rateLimit : SwerveConstants.Y_RATE_LIMIT_DICT.entrySet()) {
      if (Math.abs(getPosition() - rateLimit.getKey()) < SwerveConstants.POSITION_DIST_DELTA) {
        // Update:
        SwerveConstants.Y_ACCEL_RATE_LIMIT = rateLimit.getValue()[0];
        SwerveConstants.Y_DECEL_RATE_LIMIT = rateLimit.getValue()[1];
      }
    }
  }
}
