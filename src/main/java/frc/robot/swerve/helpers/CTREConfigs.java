// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.helpers;

import static frc.robot.Constants.PIDConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

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
    SupplyCurrentLimitConfiguration angleSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            angleEnableCurrentLimit,
            angleContinuousCurrentLimit,
            anglePeakCurrentLimit,
            anglePeakCurrentDuration);

    swerveAngleFXConfig.slot0.kP = angleKP;
    swerveAngleFXConfig.slot0.kI = angleKI;
    swerveAngleFXConfig.slot0.kD = angleKD;
    swerveAngleFXConfig.slot0.kF = angleKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            driveEnableCurrentLimit,
            driveContinuousCurrentLimit,
            drivePeakCurrentLimit,
            drivePeakCurrentDuration);

    swerveDriveFXConfig.slot0.kP = driveKP;
    swerveDriveFXConfig.slot0.kI = driveKI;
    swerveDriveFXConfig.slot0.kD = driveKD;
    swerveDriveFXConfig.slot0.kF = driveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveFXConfig.openloopRamp = openLoopRamp;
    swerveDriveFXConfig.closedloopRamp = closedLoopRamp;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
