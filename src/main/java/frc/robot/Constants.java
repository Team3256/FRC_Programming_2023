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
    public static final double stickDeadband = 0.1;

    public static final class SwerveConstants {
        public static final int pigeonID = 9;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

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
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Wheel Configurations */
        //TODO: Check with mechanical to see what gearing we are using with the swerve modules
        public static final double WHEEL_DIAMETER = 0.10033;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;
        private static final double ANGULAR_VELOCITY_CONSTANT = 1;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                DRIVE_REDUCTION *
                WHEEL_DIAMETER * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ANGULAR_VELOCITY_CONSTANT * MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // TODO: Reassign Module Motor and Cancoder ID's to the actual ones in Phoenix tuner
        public static final class Mod0 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final double angleOffset = Units.radiansToDegrees(2.977361);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final double angleOffset = Units.radiansToDegrees(0.846566);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 15;
            public static final double angleOffset = Units.radiansToDegrees(3.166089);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final double angleOffset = Units.radiansToDegrees(4.067925) - 90;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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
            public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
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
                            new Translation3d(midX, nodeFirstY + nodeSeparationY * i, isCube ? midCubeZ : midConeZ);
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
