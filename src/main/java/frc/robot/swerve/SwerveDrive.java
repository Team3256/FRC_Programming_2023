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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.helpers.Conversions;
import frc.robot.swerve.helpers.SwerveModule;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  public SwerveDriveOdometry odometry;

  private final SwerveIO swerveIO;
  private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();

  public SwerveDrive(SwerveIO swerveIO) {
    odometry =
        new SwerveDriveOdometry(
            swerveKinematics,
                swerveIO.getYaw(),
            new SwerveModulePosition[] {
             getFrontLeftModulePosition(),
                    getFrontRightModulePosition(),
                    getBackLeftModulePosition(),
                    getBackRightModulePosition()
            });
    this.swerveIO = swerveIO;
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveIO.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(swerveIO.getYaw(), getPositions(), pose);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    states[0] = getFrontLeftModulePosition();
    states[1] = getFrontRightModulePosition();
    states[2] = getBackLeftModulePosition();
    states[3] = getBackRightModulePosition();
    return states;
  }

  @Override
  public void periodic() {
    swerveIO.updateInputs(swerveIOInputs);
    Logger.getInstance().processInputs("SwerveDrive", swerveIOInputs);
    odometry.update(swerveIO.getYaw(), getPositions());
    Logger.getInstance().recordOutput("Odometry", getPose());

    for (SwerveModule mod : swerveIO.getSwerveModules()) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    }
  }

  public SwerveModulePosition getFrontLeftModulePosition() {
    double velocity = swerveIOInputs.frontLeftModuleVelocity;
    Rotation2d angle =
            Rotation2d.fromDegrees(
                    Conversions.falconToDegrees(swerveIOInputs.frontLeftModuleAngle, angleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getFrontRightModulePosition() {
    double velocity = swerveIOInputs.frontRightModuleVelocity;
    Rotation2d angle =
            Rotation2d.fromDegrees(
                    Conversions.falconToDegrees(swerveIOInputs.frontRightModuleAngle, angleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getBackLeftModulePosition() {
    double velocity = swerveIOInputs.backLeftModuleVelocity;
    Rotation2d angle =
            Rotation2d.fromDegrees(
                    Conversions.falconToDegrees(swerveIOInputs.backLeftModuleAngle, angleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public SwerveModulePosition getBackRightModulePosition() {
    double velocity = swerveIOInputs.backRightModuleVelocity;
    Rotation2d angle =
            Rotation2d.fromDegrees(
                    Conversions.falconToDegrees(swerveIOInputs.backRightModuleAngle, angleGearRatio));
    return new SwerveModulePosition(velocity, angle);
  }

  public void zeroGyro() {
    swerveIO.zeroGyro();
  }
}
