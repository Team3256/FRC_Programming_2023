// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.helpers.SwerveModuleConstants;
import java.util.Map;

public final class Constants {
  public static final boolean DEBUG = false;
  public static final double stickDeadband = 0.1;
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

  public static final class IntakeConstants {
    public static final int intakeMotorID = 1;
    public static final double kIntakeForwardSpeed = 0.5;
    public static final double kOuttakeSpeed = -0.5;
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
    public static final double lockedSpeed = 0.5;
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.86 / 1.0); // 6.86:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

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
  }

  public static final class AutoConstants {
    public static final boolean AUTO_DEBUG = false;
    public static final double TRAJECTORY_DURATION_FACTOR = 1.11;
    public static final double COMMAND_MARKER_THRESHOLD = 0.05; // meters

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
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
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    // Dimensions for community and charging station, including the tape.
    public static final class Community {
      // Region dimensions
      public static final double innerX = 0.0;
      public static final double midX =
          Units.inchesToMeters(132.375); // Tape to the left of charging station
      public static final double outerX =
          Units.inchesToMeters(193.25); // Tape to the right of charging station
      public static final double leftY = Units.feetToMeters(18.0);
      public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
      public static final double rightY = 0.0;
      public static final Translation2d[] regionCorners =
          new Translation2d[] {
            new Translation2d(innerX, rightY),
            new Translation2d(innerX, leftY),
            new Translation2d(midX, leftY),
            new Translation2d(midX, midY),
            new Translation2d(outerX, midY),
            new Translation2d(outerX, rightY),
          };

      // Charging station dimensions
      public static final double chargingStationLength = Units.inchesToMeters(76.125);
      public static final double chargingStationWidth = Units.inchesToMeters(97.25);
      public static final double chargingStationOuterX = outerX - tapeWidth;
      public static final double chargingStationInnerX =
          chargingStationOuterX - chargingStationLength;
      public static final double chargingStationLeftY = midY - tapeWidth;
      public static final double chargingStationRightY =
          chargingStationLeftY - chargingStationWidth;
      public static final Translation2d[] chargingStationCorners =
          new Translation2d[] {
            new Translation2d(chargingStationInnerX, chargingStationRightY),
            new Translation2d(chargingStationInnerX, chargingStationLeftY),
            new Translation2d(chargingStationOuterX, chargingStationRightY),
            new Translation2d(chargingStationOuterX, chargingStationLeftY)
          };

      // Cable bump
      public static final double cableBumpInnerX =
          innerX + Grids.outerX + Units.inchesToMeters(95.25);
      public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
      public static final Translation2d[] cableBumpCorners =
          new Translation2d[] {
            new Translation2d(cableBumpInnerX, 0.0),
            new Translation2d(cableBumpInnerX, chargingStationRightY),
            new Translation2d(cableBumpOuterX, 0.0),
            new Translation2d(cableBumpOuterX, chargingStationRightY)
          };
    }

    // Dimensions for grids and nodes
    public static final class Grids {
      // X layout
      public static final double outerX = Units.inchesToMeters(54.25);
      public static final double lowX =
          outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
      public static final double midX = outerX - Units.inchesToMeters(22.75);
      public static final double highX = outerX - Units.inchesToMeters(39.75);

      // Y layout
      public static final int nodeRowCount = 9;
      public static final double nodeFirstY = Units.inchesToMeters(20.19);
      public static final double nodeSeparationY = Units.inchesToMeters(22.0);

      // Z layout
      public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
      public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
      public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
      public static final double highConeZ = Units.inchesToMeters(46.0);
      public static final double midConeZ = Units.inchesToMeters(34.0);

      // Translations (all nodes in the same column/row have the same X/Y coordinate)
      public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
      public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
      public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
      public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
      public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

      static {
        for (int i = 0; i < nodeRowCount; i++) {
          boolean isCube = i == 1 || i == 4 || i == 7;
          lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
          midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
          mid3dTranslations[i] =
              new Translation3d(
                  midX, nodeFirstY + nodeSeparationY * i, isCube ? midCubeZ : midConeZ);
          high3dTranslations[i] =
              new Translation3d(
                  highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
          highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
        }
      }

      // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
      public static final double complexLowXCones =
          outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
      public static final double complexLowXCubes = lowX; // Centered X under cube nodes
      public static final double complexLowOuterYOffset =
          nodeFirstY - Units.inchesToMeters(3.0) - (Units.inchesToMeters(25.75) / 2.0);

      public static final Translation2d[] complexLowTranslations =
          new Translation2d[] {
            new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
            new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
            new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
            new Translation2d(
                complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
          };
    }

    // Dimensions for loading zone and substations, including the tape
    public static final class LoadingZone {
      // Region dimensions
      public static final double width = Units.inchesToMeters(99.0);
      public static final double innerX = FieldConstants.fieldLength;
      public static final double midX = fieldLength - Units.inchesToMeters(132.25);
      public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
      public static final double leftY = FieldConstants.fieldWidth;
      public static final double midY = leftY - Units.inchesToMeters(50.5);
      public static final double rightY = leftY - width;
      public static final Translation2d[] regionCorners =
          new Translation2d[] {
            new Translation2d(
                midX, rightY), // Start at lower left next to border with opponent community
            new Translation2d(midX, midY),
            new Translation2d(outerX, midY),
            new Translation2d(outerX, leftY),
            new Translation2d(innerX, leftY),
            new Translation2d(innerX, rightY),
          };

      // Double substation dimensions
      public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
      public static final double doubleSubstationX = innerX - doubleSubstationLength;
      public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);

      // Single substation dimensions
      public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
      public static final double singleSubstationLeftX =
          FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
      public static final double singleSubstationCenterX =
          singleSubstationLeftX + (singleSubstationWidth / 2.0);
      public static final double singleSubstationRightX =
          singleSubstationLeftX + singleSubstationWidth;
      public static final Translation2d singleSubstationTranslation =
          new Translation2d(singleSubstationCenterX, leftY);

      public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
      public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
      public static final double singleSubstationCenterZ =
          singleSubstationLowZ + (singleSubstationHeight / 2.0);
      public static final double singleSubstationHighZ =
          singleSubstationLowZ + singleSubstationHeight;
    }

    // Locations of staged game pieces
    public static final class StagingLocations {
      public static final double centerOffsetX = Units.inchesToMeters(47.36);
      public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
      public static final double firstY = Units.inchesToMeters(36.19);
      public static final double separationY = Units.inchesToMeters(48.0);
      public static final Translation2d[] translations = new Translation2d[4];

      static {
        for (int i = 0; i < translations.length; i++) {
          translations[i] = new Translation2d(positionX, firstY + (i * separationY));
        }
      }
    }

    // AprilTag locations (do not flip for red alliance)
    public static final Map<Integer, Pose3d> aprilTags =
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
