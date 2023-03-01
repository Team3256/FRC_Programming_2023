// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import static frc.robot.swerve.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        kAngleEnableCurrentLimit,
        kAngleContinuousCurrentLimit,
        kAnglePeakCurrentLimit,
        kAnglePeakCurrentDuration);

    swerveAngleFXConfig.slot0.kP = kAngleKP;
    swerveAngleFXConfig.slot0.kI = kAngleKI;
    swerveAngleFXConfig.slot0.kD = kAngleKD;
    swerveAngleFXConfig.slot0.kF = kAngleKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        kDriveEnableCurrentLimit,
        kDriveContinuousCurrentLimit,
        kDrivePeakCurrentLimit,
        kDrivePeakCurrentDuration);

    swerveDriveFXConfig.slot0.kP = kDriveKP;
    swerveDriveFXConfig.slot0.kI = kDriveKI;
    swerveDriveFXConfig.slot0.kD = kDriveKD;
    swerveDriveFXConfig.slot0.kF = kDriveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.openloopRamp = kOpenLoopRamp;
    swerveDriveFXConfig.closedloopRamp = kClosedLoopRamp;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = kCanCoderInvert;
    swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
