// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helper.AdaptiveSlewRateLimiter;
import frc.robot.swerve.helpers.SwerveModule;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeftModule = new SwerveModule(0, FrontLeft.constants);
  private final SwerveModule frontRightModule = new SwerveModule(1, FrontRight.constants);
  private final SwerveModule backLeftModule = new SwerveModule(2, BackLeft.constants);
  private final SwerveModule backRightModule = new SwerveModule(3, BackRight.constants);
  private final Field2d field = new Field2d();

  private final AdaptiveSlewRateLimiter adaptiveXRateLimiter =
      new AdaptiveSlewRateLimiter(
          Constants.SwerveConstants.X_ACCEL_RATE_LIMIT,
          Constants.SwerveConstants.X_DECEL_RATE_LIMIT);
  private final AdaptiveSlewRateLimiter adaptiveYRateLimiter =
      new AdaptiveSlewRateLimiter(
          Constants.SwerveConstants.Y_ACCEL_RATE_LIMIT,
          Constants.SwerveConstants.Y_DECEL_RATE_LIMIT);

  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // Front Right
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // Back right
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // Front left
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // Back right
          );

  private final SwerveModule[] swerveModules = {
    frontLeftModule, frontRightModule, backLeftModule, backRightModule
  };

  public SwerveDriveOdometry odometry;
  public PigeonIMU gyro;

  public SwerveDrive() {
    gyro = new PigeonIMU(pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    odometry =
        new SwerveDriveOdometry(
            swerveKinematics,
            getYaw(),
            new SwerveModulePosition[] {
              frontLeftModule.getPosition(),
              frontRightModule.getPosition(),
              backLeftModule.getPosition(),
              backRightModule.getPosition()
            });
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds.vxMetersPerSecond =
        adaptiveXRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
    chassisSpeeds.vyMetersPerSecond =
        adaptiveYRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond);

    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
            chassisSpeeds); // same as the older version of drive but takes in the calculated
    // chassisspeed
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw());
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    chassisSpeeds.vxMetersPerSecond =
        adaptiveXRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
    chassisSpeeds.vyMetersPerSecond =
        adaptiveYRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond);

    SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    for (SwerveModule mod : swerveModules) {
      // TODO: Optimize the module state using wpilib optimize method
      // TODO: Check if the optimization is happening in the setDesiredState method
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    Logger.getInstance().recordOutput("SwerveModuleStates", swerveModuleStates);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getPositions());
    Logger.getInstance().recordOutput("Odometry", getPose());

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    }
  }

  public void setTrajectory(Trajectory trajectory) {
    field.getObject("traj").setTrajectory(trajectory);
  }
}
