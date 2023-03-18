// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.swerve.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.limelight.Limelight;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;
import frc.robot.swerve.helpers.*;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements Loggable, CANTestable {
  private SwerveDrivePoseEstimator poseEstimator;

  private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();

  private final Field2d field = new Field2d();
  private final Field2d limelightLocalizationField = new Field2d();

  private final AdaptiveSlewRateLimiter adaptiveXRateLimiter =
      new AdaptiveSlewRateLimiter(kXAccelRateLimit, kXDecelRateLimit);
  private final AdaptiveSlewRateLimiter adaptiveYRateLimiter =
      new AdaptiveSlewRateLimiter(kYAccelRateLimit, kYDecelRateLimit);

  public Pigeon2 gyro;
  private final SwerveIO swerveIO;

  public SwerveDrive(SwerveIO swerveIO) {
    this.swerveIO = swerveIO;
    gyro = new Pigeon2(kPigeonID, kPigeonCanBus);
    gyro.configFactoryDefault();
    zeroGyro();

    // TODO MAKE POSITION 0,0
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            swerveIO.getYaw(),
            new SwerveModulePosition[] {
              getFrontLeftModulePosition(),
              getFrontRightModulePosition(),
              getBackLeftModulePosition(),
              getBackRightModulePosition()
            },
            new Pose2d());

    if (Constants.kDebugEnabled) {
      SmartDashboard.putData("Limelight Localization Field", limelightLocalizationField);
      SmartDashboard.putData("Field", field);
    }
    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      mod.resetToAbsolute();
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
    Pose2d robotPoseVelocity =
        new Pose2d(
            chassisSpeeds.vxMetersPerSecond * kPeriodicDeltaTime,
            chassisSpeeds.vyMetersPerSecond * kPeriodicDeltaTime,
            Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * kPeriodicDeltaTime));
    Twist2d twistVelocity = (new Pose2d()).log(robotPoseVelocity);
    ChassisSpeeds updatedChassisSpeeds =
        new ChassisSpeeds(
            twistVelocity.dx / kPeriodicDeltaTime,
            twistVelocity.dy / kPeriodicDeltaTime,
            twistVelocity.dtheta / kPeriodicDeltaTime);

    if (FeatureFlags.kSwerveAccelerationLimitingEnabled) {
      chassisSpeeds.vxMetersPerSecond =
          adaptiveXRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
      chassisSpeeds.vyMetersPerSecond =
          adaptiveYRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
    }

    SwerveModuleState[] swerveModuleStates =
        kSwerveKinematics.toSwerveModuleStates(updatedChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    Logger.getInstance().recordOutput("SwerveModuleStates", swerveModuleStates);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds swerveChassisSpeed =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getPose().getRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    drive(swerveChassisSpeed, isOpenLoop);
  }

  public void setDesiredAngleState(SwerveModuleState[] swerveModuleStates) {
    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      mod.setDesiredAngleState(swerveModuleStates[mod.moduleNumber]);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);

    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(swerveIO.getYaw(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  public void zeroGyro() {
    swerveIO.zeroGyro();
  }

  public void setGyro(double yawDegrees) {
    gyro.setYaw(yawDegrees);
  }

  public boolean shouldAddVisionMeasurement(
      Pose2d limelightPose,
      double LimelightTranslationThresholdMeters,
      double LimelightRotationThreshold) {
    Pose2d relativePose = limelightPose.relativeTo(poseEstimator.getEstimatedPosition());
    return Math.abs(relativePose.getTranslation().getNorm()) < LimelightTranslationThresholdMeters
        && Math.abs(relativePose.getRotation().getRadians()) < LimelightRotationThreshold;
  }

  public void localize(
      String networkTablesName,
      double fieldTransformOffsetX,
      double fieldTransformOffsetY,
      double LimelightTranslationThresholdMeters,
      double LimelightRotationThreshold) {
    if (Limelight.hasValidTargets(networkTablesName)) {
      double[] visionBotPose = Limelight.getBotpose(networkTablesName);

      if (visionBotPose.length != 0) {
        double tx = visionBotPose[0] + fieldTransformOffsetX;
        double ty = visionBotPose[1] + fieldTransformOffsetY;

        // botpose from network tables uses degrees, not radians, so need to convert
        double rx = visionBotPose[3];
        double ry = visionBotPose[4];
        double rz = ((visionBotPose[5] + 360) % 360);

        double tl = Limelight.getLatency_Pipeline(networkTablesName);

        Pose2d limelightPose = new Pose2d(new Translation2d(tx, ty), Rotation2d.fromDegrees(rz));

        if (shouldAddVisionMeasurement(
            limelightPose, LimelightTranslationThresholdMeters, LimelightRotationThreshold)) {
          poseEstimator.addVisionMeasurement(
              limelightPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl));
        }

        if (Constants.kDebugEnabled) {
          limelightLocalizationField.setRobotPose(limelightPose);
          SmartDashboard.putNumber("Lime Light pose x", limelightPose.getX());
          SmartDashboard.putNumber("Lime Light pose y", limelightPose.getY());
          SmartDashboard.putNumber(
              "Lime Light pose theta", limelightPose.getRotation().getDegrees());
        }
      }
    }
  }

  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (kInvertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
  }

  @Override
  public void periodic() {
    swerveIO.updateInputs(swerveIOInputs);
    Logger.getInstance().processInputs("SwerveDrive", swerveIOInputs);
    poseEstimator.update(swerveIO.getYaw(), getPositions());
    SmartDashboard.putNumber("Gyro Angle", swerveIO.getYaw().getDegrees());
    Logger.getInstance().recordOutput("Odometry", getPose());
    if (Constants.kDebugEnabled) {
      field.setRobotPose(poseEstimator.getEstimatedPosition());
      Logger.getInstance().recordOutput("Odometry", getPose());

      for (SwerveModule mod : swerveIO.getSwerveModules()) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      }
    }

    this.localize(
        FrontConstants.kLimelightNetworkTablesName,
        FrontConstants.kFieldTranslationOffsetX,
        FrontConstants.kFieldTranslationOffsetY,
        FrontConstants.kLimelightTranslationThresholdMeters,
        FrontConstants.kLimelightRotationThreshold);
    this.localize(
        SideConstants.kLimelightNetworkTablesName,
        SideConstants.kFieldTranslationOffsetX,
        SideConstants.kFieldTranslationOffsetY,
        SideConstants.kLimelightTranslationThresholdMeters,
        SideConstants.kLimelightRotationThreshold);
    this.localize(
        BackConstants.kLimelightNetworkTablesName,
        BackConstants.kFieldTranslationOffsetX,
        BackConstants.kFieldTranslationOffsetY,
        BackConstants.kLimelightTranslationThresholdMeters,
        BackConstants.kLimelightTranslationThresholdMeters);
  }

  public void setTrajectory(Trajectory trajectory) {
    field.getObject("traj").setTrajectory(trajectory);
  }

  @Override
  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add("gyro", new DoubleSendable(gyro::getYaw, "Gyro"));
    for (int i = 0; i < swerveIO.getSwerveModules().length; i++) {
      getLayout(kDriverTabName)
          .add("Encoder " + i, swerveIO.getSwerveModules()[i].getAngleEncoder());
    }
    for (int i = 0; i < swerveIO.getSwerveModules().length; i++) {
      swerveIO.getSwerveModules()[i].logInit();
    }
  }

  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab)
        .getLayout(kSwerveLayoutName, BuiltInLayouts.kList)
        .withSize(2, 4);
  }

  public boolean CANTest() {
    System.out.println("Testing drivetrain CAN:");
    boolean result = true;
    for (SwerveModule device : swerveIO.getSwerveModules()) {
      result &= device.test();
    }
    result &= CANDeviceTester.testPigeon(gyro);
    System.out.println("Drivetrain CAN connected: " + result);
    getLayout(kElectricalTabName).add("Drivetrain CAN connected", result);
    return result;
  }

  public void setDriveMotorsNeutralMode(NeutralMode neutralMode) {
    for (SwerveModule swerveModule : swerveIO.getSwerveModules()) {
      swerveModule.setDriveMotorNeutralMode(neutralMode);
    }
  }

  public void setAngleMotorsNeutralMode(NeutralMode neutralMode) {
    for (SwerveModule swerveModule : swerveIO.getSwerveModules()) {
      swerveModule.setAngleMotorNeutralMode(neutralMode);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    states[0] = getFrontLeftModulePosition();
    states[1] = getFrontRightModulePosition();
    states[2] = getBackLeftModulePosition();
    states[3] = getBackRightModulePosition();
    return states;
  }

  public SwerveModulePosition getFrontLeftModulePosition() {
    double velocity = swerveIOInputs.frontLeftModuleVelocity;
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(swerveIOInputs.frontLeftModuleAngle, kAngleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getFrontRightModulePosition() {
    double velocity = swerveIOInputs.frontRightModuleVelocity;
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(swerveIOInputs.frontRightModuleAngle, kAngleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getBackLeftModulePosition() {
    double velocity = swerveIOInputs.backLeftModuleVelocity;
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(swerveIOInputs.backLeftModuleAngle, kAngleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getBackRightModulePosition() {
    double velocity = swerveIOInputs.backRightModuleVelocity;
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(swerveIOInputs.backRightModuleAngle, kAngleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }
}
