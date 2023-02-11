// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.helpers;

import static frc.robot.Constants.*;
import static frc.robot.auto.AutoConstants.kTranslationFF;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveController {
  PIDController xPositionController;
  PIDController yPositionController;

  ProfiledPIDController thetaController;
  private Pose2d poseTolerance = new Pose2d();
  private Pose2d poseError;
  private Rotation2d rotationError;
  private double prevCurrentRotation;
  private boolean firstIteration = true;

  public SwerveDriveController(
      PIDController xPositionController,
      PIDController yPositionController,
      ProfiledPIDController thetaController) {
    this.xPositionController = xPositionController;
    this.yPositionController = yPositionController;
    this.thetaController = thetaController;
  }

  public boolean atReference() {
    final Translation2d eTranslate = poseError.getTranslation();
    final Rotation2d eRotate = rotationError;
    final Translation2d tolTranslate = poseTolerance.getTranslation();
    final Rotation2d tolRotate = poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  public ChassisSpeeds calculate(
      Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
    // If this is the first run, then we need to reset the theta controller to the
    // current pose's
    // heading.
    if (firstIteration) {
      thetaController.reset(currentPose.getRotation().getRadians());
      prevCurrentRotation = currentPose.getRotation().getRadians();
      firstIteration = false;
    }

    double currentRotation = currentPose.getRotation().getRadians();

    if (Math.abs(currentRotation - prevCurrentRotation) >= Math.PI) {
      if (prevCurrentRotation > 0) {
        currentRotation = prevCurrentRotation + Math.abs(-Math.PI - currentRotation);
      } else {
        currentRotation = prevCurrentRotation + Math.abs(Math.PI - currentRotation);
      }
    }

    if (kDebugEnabled) {
      SmartDashboard.putNumber("Current Rotation", currentRotation * 180 / Math.PI);
      SmartDashboard.putNumber("Current Pose Rotation", currentPose.getRotation().getDegrees());
      SmartDashboard.putNumber(
          "Current Rotation Error", (currentRotation * 180 / Math.PI) - angleRef.getDegrees());
    }

    // Calculate feedforward velocities (field-relative).
    double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos() * kTranslationFF;
    double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin() * kTranslationFF;

    double thetaFF = thetaController.calculate(currentRotation, angleRef.getRadians());

    if (kDebugEnabled) {
      SmartDashboard.putNumber("Theta Current", currentRotation * 180 / Math.PI);
      SmartDashboard.putNumber("Theta Setpoint", angleRef.getDegrees());
    }

    poseError = poseRef.relativeTo(currentPose);
    rotationError = angleRef.minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xPositionController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = yPositionController.calculate(currentPose.getY(), poseRef.getY());

    prevCurrentRotation = currentRotation;

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  }

  public void reset() {
    firstIteration = true;
  }
}
