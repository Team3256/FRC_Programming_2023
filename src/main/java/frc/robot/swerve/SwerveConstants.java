// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.helpers.SwerveModuleConstants;

public final class SwerveConstants {

  /* Angle Motor PID Values */
  public static final double kAngleKP = 0.6;
  public static final double kAngleKI = 0.0;
  public static final double kAngleKD = 12.0;
  public static final double kAngleKF = 0.0;

  /* Drive Motor PID Values */
  public static final double kDriveKP = 0.10;
  public static final double kDriveKI = 0.0;
  public static final double kDriveKD = 0.0;
  public static final double kDriveKF = 0.0;

  /* Drive Motor Characterization Values */
  public static final double kDriveKS =
      (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
  public static final double kDriveKV = (2.44 / 12);
  public static final double kDriveKA = (0.27 / 12);

  public static final int kLongCANTimeoutMs = 1;
  public static final double kMaxDriveVoltage = 1;
  public static final double kMk4DriveVelocityKp = 1;
  public static final double kMk4DriveVelocityKi = 1;
  public static final double kMk4DriveVelocityKd = 1;
  public static final double kMk4DriveVelocityKf = 1;

  public static final double kMk4AziKp = 1;
  public static final double kMk4AziKi = 1;
  public static final double kMk4AziKd = 1;

  public static final double kDriveReduction = 1;
  public static final double kSteerReduction = 1;

  public static final int kPigeonID = 1;
  public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

  /* Drivetrain Constants */
  // TODO: Update these constants later
  public static final double kTrackWidth = Units.inchesToMeters(21.73); // 0.4445 in 2022 Constants
  public static final double kWheelBase = Units.inchesToMeters(21.73); // 0.4445 in 2022 Constants
  public static final double kWheelDiameter = Units.inchesToMeters(3.94);
  public static final double kWheelCircumference = kWheelDiameter * Math.PI;

  public static final double kOpenLoopRamp = 0.25;
  public static final double kClosedLoopRamp = 0.0;

  public static final double kDriveGearRatio = (6.86); // 6.86:1
  public static final double kAngleGearRatio = (12.8); // 12.8:1

  public static final double[] kLockAngleOffsets = {0, 3 * Math.PI / 2, Math.PI / 2, Math.PI};

  public static final SwerveDriveKinematics kSwerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

  /* Swerve Current Limiting */
  public static final int kAngleContinuousCurrentLimit = 25;
  public static final int kAnglePeakCurrentLimit = 40;
  public static final double kAnglePeakCurrentDuration = 0.1;
  public static final boolean kAngleEnableCurrentLimit = true;

  public static final int kDriveContinuousCurrentLimit = 35;
  public static final int kDrivePeakCurrentLimit = 60;
  public static final double kDrivePeakCurrentDuration = 0.1;
  public static final boolean kDriveEnableCurrentLimit = true;

  /* Swerve Profiling Values */
  public static final double kMaxSpeed = 4.5; // meters per second
  public static final double kMaxAngularVelocity = 11.5;

  /* Neutral Modes */
  public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
  public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

  /* Motor Inverts */
  public static final boolean kDriveMotorInvert = false;
  public static final boolean kAngleMotorInvert = false;

  /* Angle Encoder Invert */
  public static final boolean kCanCoderInvert = false;

  /* Module Specific Constants */
  public static final class FrontLeft {
    public static final int kDriveMotorID = 3;
    public static final int kAngleMotorID = 4;
    public static final int kCanCoderID = 2;
    public static final double kAngleOffset = 531.6064455; // 531 or 171 (ziptide constants)
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class FrontRight {
    public static final int kDriveMotorID = 6;
    public static final int kAngleMotorID = 7;
    public static final int kCanCoderID = 5;
    public static final double kAngleOffset = 48.691406; // (ziptide constants)
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class BackLeft {
    public static final int kDriveMotorID = 9;
    public static final int kAngleMotorID = 10;
    public static final int kCanCoderID = 8;
    public static final double kAngleOffset = 174.770508; // (ziptide constants)
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class BackRight {
    public static final int kDriveMotorID = 12;
    public static final int kAngleMotorID = 13;
    public static final int kCanCoderID = 11;
    public static final double angleOffset = 233.0419925; // (ziptide constants)
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, angleOffset);
  }

  public static final double kSensitivityScale = 0.20;
}
