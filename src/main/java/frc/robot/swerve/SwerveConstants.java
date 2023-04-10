// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.helpers.COTSFalconSwerveConstants;
import frc.robot.swerve.helpers.SwerveModuleConstants;

public final class SwerveConstants {
  public static final int kPigeonID = 1;
  public static final String kPigeonCanBus = "mani";

  public static final boolean kFieldRelative = true;
  public static final double kPeriodicDeltaTime = 0.02;
  public static final boolean kOpenLoop = true;
  public static final double kSwervePitTestSpeed = 0.3;

  /* Meters per second squared */
  public static int kXAccelRateLimit = 8;
  public static int kXDecelRateLimit = 14;

  public static int kYAccelRateLimit = 8;
  public static int kYDecelRateLimit = 14;

  public static double kAutoEngageLimitVelocity = 1;

  public static final COTSFalconSwerveConstants kChosenModule =
      COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

  /* Drivetrain Constants */
  public static final double kTrackWidth = Units.inchesToMeters(20.75);
  public static final double kWheelBase = Units.inchesToMeters(20.75);
  public static final double kWheelDiameter = 0.1016;
  public static final double kWheelCircumference = kWheelDiameter * Math.PI;

  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static final SwerveDriveKinematics kSwerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
          new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

  /* Module Gear Ratios */
  public static final double kDriveGearRatio = kChosenModule.driveGearRatio;
  public static final double kAngleGearRatio = kChosenModule.angleGearRatio;

  /* Motor Inverts */
  public static final boolean kAngleMotorInvert = kChosenModule.angleMotorInvert;
  public static final boolean kDriveMotorInvert = kChosenModule.driveMotorInvert;

  /* Angle Encoder Invert */
  public static final boolean kCanCoderInvert = kChosenModule.canCoderInvert;

  /* Swerve Current Limiting */
  public static final int kAngleContinuousCurrentLimit = 25;
  public static final int kAnglePeakCurrentLimit = 40;
  public static final double kAnglePeakCurrentDuration = 0.1;
  public static final boolean kAngleEnableCurrentLimit = true;

  public static final int kDriveContinuousCurrentLimit = 35;
  public static final int kDrivePeakCurrentLimit = 60;
  public static final double kDrivePeakCurrentDuration = 0.1;
  public static final boolean kDriveEnableCurrentLimit = true;

  /*
   * These values are used by the drive falcon to ramp in open loop and closed
   * loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
   */
  public static final double kOpenLoopRamp = 0.25;
  public static final double kClosedLoopRamp = 0.0;

  /* Angle Motor PID Values */
  public static final double kAngleKP = kChosenModule.angleKP;
  public static final double kAngleKI = kChosenModule.angleKI;
  public static final double kAngleKD = kChosenModule.angleKD;
  public static final double kAngleKF = kChosenModule.angleKF;

  /* Drive Motor PID Values */
  public static final double kDriveKP = 0.05; // TODO: This must be tuned to specific robot
  public static final double kDriveKI = 0.0;
  public static final double kDriveKD = 0.0;
  public static final double kDriveKF = 0.0;

  public static final double[] klockAngleOffsetsHorizontal = {
    Math.PI / 4, Math.PI / 4, Math.PI / 4, Math.PI / 4
  }; // All modules turn left to lock
  public static final double[] kLockAngleOffsetsX = {0, 3 * Math.PI / 2, Math.PI / 2, Math.PI};

  /*
   * Drive Motor Characterization Values
   * Divide SYSID values by 12 to convert from volts to percent output for CTRE
   */
  public static final double kDriveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
  public static final double kDriveKV = (1.51 / 12);
  public static final double kDriveKA = (0.27 / 12);

  /* Swerve Profiling Values */
  /** Meters per Second */
  public static final double kMaxSpeed = 4.96824; // TODO: This must be tuned to specific robot
  /** Radians per Second */
  public static final double kMaxAngularVelocity = 2 * Math.PI;

  /* Neutral Modes */
  public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
  public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
  public static final boolean kInvertGyroYaw = false; // Always ensure Gyro is CCW+ CW-

  /* PID Constants Trapezoid Profile for the Azimuth Control */
  public static final double kAzimuthP = 0.15;
  public static final double kAzimuthI = 0.00;
  public static final double kAzimuthD = 0.01;
  public static final Rotation2d[] kAlphaOffsets = {
    Rotation2d.fromDegrees(268.4179), // Front Left - 0
    Rotation2d.fromDegrees(314.9192), // Front Right - 1
    Rotation2d.fromDegrees(77.6953), // Back Left - 2
    Rotation2d.fromDegrees(280.7226) // Back Right - 3
  };

  public static final Rotation2d[] kZiptideOffsets = {
    Rotation2d.fromDegrees(531.6064455), // Front Left - 0
    Rotation2d.fromDegrees(48.691406), // Front Right - 1
    Rotation2d.fromDegrees(174.770508), // Back Left - 2
    Rotation2d.fromDegrees(233.0419925) // Back Right - 3
  };

  /* Module Specific Constants */
  public static final class FrontLeft {
    public static final int kDriveMotorID = 3;
    public static final int kAngleMotorID = 4;
    public static final int kCanCoderID = 2;
    public static final Rotation2d kAngleOffset = kRobotType.getOffset(0);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class FrontRight {
    public static final int kDriveMotorID = 6;
    public static final int kAngleMotorID = 7;
    public static final int kCanCoderID = 5;
    public static final Rotation2d kAngleOffset = kRobotType.getOffset(1);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class BackLeft {
    public static final int kDriveMotorID = 9;
    public static final int kAngleMotorID = 10;
    public static final int kCanCoderID = 8;
    public static final Rotation2d kAngleOffset = kRobotType.getOffset(2);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, kAngleOffset);
  }

  public static final class BackRight {
    public static final int kDriveMotorID = 12;
    public static final int kAngleMotorID = 13;
    public static final int kCanCoderID = 11;
    public static final Rotation2d angleOffset = kRobotType.getOffset(3);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCanCoderID, angleOffset);
  }

  public static final double kSensitivityScale = 0.3;

  /* AutoBalance */
  public static double kXAutoBalanceVelocity = 0.7;
  public static final Rotation2d kAutoBalanceMaxError = new Rotation2d(Units.degreesToRadians(10));
}
