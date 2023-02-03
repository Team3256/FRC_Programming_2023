// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public final class Constants {
  public static final boolean DEBUG = false;
  public static final boolean INTAKE = true;
  public static final boolean ELEVATOR = true;
  public static final boolean SWERVE = true;
  public static final double kStickDeadband = 0.1;
  public static final double azimuthStickDeadband = 0.3;
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

  public static final boolean fieldRelative = true;
  public static final boolean openLoop = true;

  public static final class IntakeConstants {
    public static final int intakeMotorID = 14;
    public static final double kIntakeConeSpeed = 0.5;
    public static final double kIntakeCubeSpeed = -0.5;
  }

  public static final class ArmConstants {
    // TODO: Fix these to comply to the mechanical ppls kg
    public static final int ARM_MOTOR_ID = -1;
    public static final double kArmGearing = 1;
    public static final double kArmInertia =
        1; // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/SingleJointedArmSim.html#%3Cinit%3E(edu.wpi.first.math.system.plant.DCMotor,double,double,double,double,double,double,boolean)
    public static final double kArmLengthMeters = 1;
    public static final double kMinAngleRads = 1;
    public static final double kMaxAngleRads = 1;
    public static final double kArmMassKg = 1;
    public static final boolean kArmSimGravity = true;

    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;
    public static final double kFF = 1;
  }

  public static final class SwerveConstants {

    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    // TODO: Update Constants
    public static final double trackWidth = Units.inchesToMeters(24.25);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.86 / 1.0); // 6.86:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final double[] lockAngleOffsets = {0, 3 * Math.PI / 2, Math.PI / 2, Math.PI};

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
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
    public static final double maxAngularVelocity = Math.PI * 1.5;
    public static final double maxAngularAcceleration = Math.PI * 0.5;

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* PID Constants Trapezoid Profile for the Azimuth Control */
    public static final double kAzimuthP = 0.09;
    public static final double kAzimuthI = 0.00;
    public static final double kAzimuthD = 0.01;

    /* Module Specific Constants */
    public static final class FrontLeft {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final double angleOffset = 531.6064455; // 531 or 171 (ziptide constants)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final class FrontRight {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 5;
      public static final double angleOffset = 48.691406; // (ziptide constants)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final class BackLeft {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 8;
      public static final double angleOffset = 174.770508; // (ziptide constants)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final class BackRight {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 11;
      public static final double angleOffset = 233.0419925; // (ziptide constants)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final double kSensitivityScale = 0.20;
  }

  public static final class AutoConstants {
    public static final boolean AUTO_DEBUG = false;
    public static final double COMMAND_MARKER_THRESHOLD = 0.05; // meters

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Max velocity
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    // Max acceleration
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PIDConstants {

    /* Angle Motor PID Values */
    public static final double angleKP = 0.6;
    public static final double angleKI = 0.0;
    public static final double angleKD = 12.0;
    public static final double angleKF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.10;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS =
        (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
    public static final double driveKV = (2.44 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Auto translation constants */

    public static double kAutoXTranslationP = 2.2;
    public static double kAutoXTranslationI = 0.025;
    public static double kAutoXTranslationD = 0;

    public static double kAutoYTranslationP = 2.2;
    public static double kAutoYTranslationI = 0.025;
    public static double kAutoYTranslationD = 0;

    public static double TRANSLATION_FF = 0.3;

    /* ThetaController constants */
    public static double kAutoThetaControllerP = 5.4;
    public static double kAutoThetaControllerI = 0.02;
    public static double kAutoThetaControllerD = 1.5;
    public static TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  }


  public static final class FieldConstants {
    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);
    public static final double kTapeWidth = Units.inchesToMeters(2.0);

    // Dimensions for community and charging station, including the tape.
    public static final class Community {
      // Region dimensions
      public static final double kInnerX = 0.0;
      public static final double kMidX =
          Units.inchesToMeters(132.375); // Tape to the left of charging station
      public static final double kOuterX =
          Units.inchesToMeters(193.25); // Tape to the right of charging station
      public static final double kLeftY = Units.feetToMeters(18.0);
      public static final double kMidY = kLeftY - Units.inchesToMeters(59.39) + kTapeWidth;
      public static final double kRightY = 0.0;
      public static final Translation2d[] kRegionCorners =
          new Translation2d[] {
            new Translation2d(kInnerX, kRightY),
            new Translation2d(kInnerX, kLeftY),
            new Translation2d(kMidX, kLeftY),
            new Translation2d(kMidX, kMidY),
            new Translation2d(kOuterX, kMidY),
            new Translation2d(kOuterX, kRightY),
          };

      // Charging station dimensions
      public static final double kChargingStationLength = Units.inchesToMeters(76.125);
      public static final double kChargingStationWidth = Units.inchesToMeters(97.25);
      public static final double kChargingStationOuterX = kOuterX - kTapeWidth;
      public static final double kChargingStationInnerX =
          kChargingStationOuterX - kChargingStationLength;
      public static final double kChargingStationLeftY = kMidY - kTapeWidth;
      public static final double kChargingStationRightY =
          kChargingStationLeftY - kChargingStationWidth;
      public static final Translation2d[] kChargingStationCorners =
          new Translation2d[] {
            new Translation2d(kChargingStationInnerX, kChargingStationRightY),
            new Translation2d(kChargingStationInnerX, kChargingStationLeftY),
            new Translation2d(kChargingStationOuterX, kChargingStationRightY),
            new Translation2d(kChargingStationOuterX, kChargingStationLeftY)
          };

      // Cable bump
      public static final double kCableBumpInnerX =
          kInnerX + Grids.kOuterX + Units.inchesToMeters(95.25);
      public static final double kCableBumpOuterX = kCableBumpInnerX + Units.inchesToMeters(7);
      public static final Translation2d[] kCableBumpCorners =
          new Translation2d[] {
            new Translation2d(kCableBumpInnerX, 0.0),
            new Translation2d(kCableBumpInnerX, kChargingStationRightY),
            new Translation2d(kCableBumpOuterX, 0.0),
            new Translation2d(kCableBumpOuterX, kChargingStationRightY)
          };
    }

    // Dimensions for grids and nodes
    public static final class Grids {
      // X layout
      public static final double kOuterX = Units.inchesToMeters(54.25);
      public static final double kLowX =
          kOuterX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
      public static final double kMidX = kOuterX - Units.inchesToMeters(22.75);
      public static final double kHighX = kOuterX - Units.inchesToMeters(39.75);

      // Y layout
      public static final int kNodeRowCount = 9;
      public static final double kNodeFirstY = Units.inchesToMeters(20.19);
      public static final double kNodeSeparationY = Units.inchesToMeters(22.0);

      // Z layout
      public static final double kCubeEdgeHigh = Units.inchesToMeters(3.0);
      public static final double kHighCubeZ = Units.inchesToMeters(35.5) - kCubeEdgeHigh;
      public static final double kMidCubeZ = Units.inchesToMeters(23.5) - kCubeEdgeHigh;
      public static final double kHighConeZ = Units.inchesToMeters(46.0);
      public static final double kMidConeZ = Units.inchesToMeters(34.0);

      // Translations (all nodes in the same column/row have the same X/Y coordinate)
      public static final Translation2d[] kLowTranslations = new Translation2d[kNodeRowCount];
      public static final Translation2d[] kMidTranslations = new Translation2d[kNodeRowCount];
      public static final Translation3d[] kMid3dTranslations = new Translation3d[kNodeRowCount];
      public static final Translation2d[] kHighTranslations = new Translation2d[kNodeRowCount];
      public static final Translation3d[] kHigh3dTranslations = new Translation3d[kNodeRowCount];

      static {
        for (int i = 0; i < kNodeRowCount; i++) {
          boolean isCube = i == 1 || i == 4 || i == 7;
          kLowTranslations[i] = new Translation2d(kLowX, kNodeFirstY + kNodeSeparationY * i);
          kMidTranslations[i] = new Translation2d(kMidX, kNodeFirstY + kNodeSeparationY * i);
          kMid3dTranslations[i] =
              new Translation3d(
                  kMidX, kNodeFirstY + kNodeSeparationY * i, isCube ? kMidCubeZ : kMidConeZ);
          kHigh3dTranslations[i] =
              new Translation3d(
                  kHighX, kNodeFirstY + kNodeSeparationY * i, isCube ? kHighCubeZ : kHighConeZ);
          kHighTranslations[i] = new Translation2d(kHighX, kNodeFirstY + kNodeSeparationY * i);
        }
      }

      // Complex low layout (shifted to account for cube vs cone rows and wide edge
      // nodes)

      public static final double kComplexLowXCones =
          kOuterX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone
      // nodes
      public static final double kComplexLowXCubes = kLowX; // Centered X under cube nodes
      public static final double kComplexLowOuterYOffset =
          kNodeFirstY - Units.inchesToMeters(3.0) - (Units.inchesToMeters(25.75) / 2.0);

      public static final Translation2d[] kComplexLowTranslations =
          new Translation2d[] {
            new Translation2d(kComplexLowXCones, kNodeFirstY - kComplexLowOuterYOffset),
            new Translation2d(kComplexLowXCubes, kNodeFirstY + kNodeSeparationY * 1),
            new Translation2d(kComplexLowXCones, kNodeFirstY + kNodeSeparationY * 2),
            new Translation2d(kComplexLowXCones, kNodeFirstY + kNodeSeparationY * 3),
            new Translation2d(kComplexLowXCubes, kNodeFirstY + kNodeSeparationY * 4),
            new Translation2d(kComplexLowXCones, kNodeFirstY + kNodeSeparationY * 5),
            new Translation2d(kComplexLowXCones, kNodeFirstY + kNodeSeparationY * 6),
            new Translation2d(kComplexLowXCubes, kNodeFirstY + kNodeSeparationY * 7),
            new Translation2d(
                kComplexLowXCones, kNodeFirstY + kNodeSeparationY * 8 + kComplexLowOuterYOffset),
          };
    }

    // Dimensions for loading zone and substations, including the tape
    public static final class LoadingZone {
      // Region dimensions
      public static final double kWidth = Units.inchesToMeters(99.0);
      public static final double kInnerX = FieldConstants.kFieldLength;
      public static final double kMidX = kFieldLength - Units.inchesToMeters(132.25);
      public static final double kOuterX = kFieldLength - Units.inchesToMeters(264.25);
      public static final double kLeftY = FieldConstants.kFieldWidth;
      public static final double kMidY = kLeftY - Units.inchesToMeters(50.5);
      public static final double kRightY = kLeftY - kWidth;
      public static final Translation2d[] kRegionCorners =
          new Translation2d[] {
            new Translation2d(
                kMidX, kRightY), // Start at lower left next to border with opponent community
            new Translation2d(kMidX, kMidY),
            new Translation2d(kOuterX, kMidY),
            new Translation2d(kOuterX, kLeftY),
            new Translation2d(kInnerX, kLeftY),
            new Translation2d(kInnerX, kRightY),
          };

      // Double substation dimensions
      public static final double kDoubleSubstationLength = Units.inchesToMeters(14.0);
      public static final double kDoubleSubstationX = kInnerX - kDoubleSubstationLength;
      public static final double kDoubleSubstationShelfZ = Units.inchesToMeters(37.375);

      // Single substation dimensions
      public static final double kSingleSubstationWidth = Units.inchesToMeters(22.75);
      public static final double kSingleSubstationLeftX =
          FieldConstants.kFieldLength - kDoubleSubstationLength - Units.inchesToMeters(88.77);
      public static final double kSingleSubstationCenterX =
          kSingleSubstationLeftX + (kSingleSubstationWidth / 2.0);
      public static final double kSingleSubstationRightX =
          kSingleSubstationLeftX + kSingleSubstationWidth;
      public static final Translation2d kSingleSubstationTranslation =
          new Translation2d(kSingleSubstationCenterX, kLeftY);

      public static final double kSingleSubstationHeight = Units.inchesToMeters(18.0);
      public static final double kSingleSubstationLowZ = Units.inchesToMeters(27.125);
      public static final double kSingleSubstationCenterZ =
          kSingleSubstationLowZ + (kSingleSubstationHeight / 2.0);
      public static final double kSingleSubstationHighZ =
          kSingleSubstationLowZ + kSingleSubstationHeight;
    }

    // Locations of staged game pieces
    public static final class StagingLocations {
      public static final double kCenterOffsetX = Units.inchesToMeters(47.36);
      public static final double kPositionX = kFieldLength / 2.0 - Units.inchesToMeters(47.36);
      public static final double kFirstY = Units.inchesToMeters(36.19);
      public static final double kSeparationY = Units.inchesToMeters(48.0);
      public static final Translation2d[] kTranslations = new Translation2d[4];

      static {
        for (int i = 0; i < kTranslations.length; i++) {
          kTranslations[i] = new Translation2d(kPositionX, kFirstY + (i * kSeparationY));
        }
      }
    }

    // AprilTag locations (do not flip for red alliance)
    public static final Map<Integer, Pose3d> kAprilTags =
        Map.of(
            1,
            new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(42.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI)),
            2,
            new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI)),
            3,
            new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI)),
            4,
            new Pose3d(
                Units.inchesToMeters(636.96),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d(0.0, 0.0, Math.PI)),
            5,
            new Pose3d(
                Units.inchesToMeters(14.25),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d()),
            6,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d()),
            7,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d()),
            8,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(42.19),
                Units.inchesToMeters(18.22),
                new Rotation3d()));
  }
}
