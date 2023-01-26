package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.helpers.SwerveModuleConstants;

public final class SwerveDriveConstants {
  public static final int kLongCANTimeoutMs = 1;
  public static final double kMaxDriveVoltage = 1;
  public static final double kMk4DriveVelocityKp = 1;
  public static final double kMk4DriveVelocityKi = 1;
  public static final double kMk4DriveVelocityKd = 1;
  public static final double kMk4DriveVelocityKf = 1;

  public static final double kMk4AziKp = 1;
  public static final double kMk4AziKi = 1;
  public static final double kMk4AziKd = 1;

  public static final double driveReduction = 1;
  public static final double steerReduction = 1;

  public static final int pigeonID = 1;
  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  /* Drivetrain Constants */
  // TODO: Update these constants later
  public static final double trackWidth = Units.inchesToMeters(21.73); // 0.4445 in 2022 Constants
  public static final double wheelBase = Units.inchesToMeters(21.73); // 0.4445 in 2022 Constants
  public static final double wheelDiameter = Units.inchesToMeters(3.94);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  public static final double driveGearRatio = (6.86 / 1.0); // 6.86:1
  public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 25;
  public static final int anglePeakCurrentLimit = 40;
  public static final double anglePeakCurrentDuration = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  public static final int driveContinuousCurrentLimit = 35;
  public static final int drivePeakCurrentLimit = 60;
  public static final double drivePeakCurrentDuration = 0.1;
  public static final boolean driveEnableCurrentLimit = true;

  /* Swerve Profiling Values */
  public static final double maxSpeed = 4.5; // meters per second
  public static final double maxAngularVelocity = 11.5;

  /* Neutral Modes */
  public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
  public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

  /* Motor Inverts */
  public static final boolean driveMotorInvert = false;
  public static final boolean angleMotorInvert = false;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = false;

  /* Module Specific Constants */
  public static final class FrontLeft {
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 2;
    public static final double angleOffset = 531.6064455; // 531 or 171 (ziptide constants)
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  public static final class FrontRight {
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 5;
    public static final double angleOffset = 48.691406; // (ziptide constants)
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  public static final class BackLeft {
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 8;
    public static final double angleOffset = 174.770508; // (ziptide constants)
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  public static final class BackRight {
    public static final int driveMotorID = 12;
    public static final int angleMotorID = 13;
    public static final int canCoderID = 11;
    public static final double angleOffset = 233.0419925; // (ziptide constants)
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID, angleOffset);
  }

  public static final double kSensitivityScale = 0.20;
}
