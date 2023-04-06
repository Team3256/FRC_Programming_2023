// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.*;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerveWithAzimuth extends CommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier rotationXAxis;
  private DoubleSupplier rotationYAxis;
  private BooleanSupplier manualRotating;
  private PIDController azimuthController;

  /** Driver control */
  public TeleopSwerveWithAzimuth(
      SwerveDrive swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationXAxis,
      DoubleSupplier rotationYAxis,
      BooleanSupplier manualRotating,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.manualRotating = manualRotating;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.azimuthController = new PIDController(kAzimuthP, kAzimuthI, kAzimuthD);
    azimuthController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    System.out.println("TeleopSwerveWithAzimuth started");
  }

  @Override
  public void execute() {
    // Obtains axis values for x and y for translation command
    double yAxis = -translationAxis.getAsDouble();
    double xAxis = -strafeAxis.getAsDouble();

    // Gets position of joystick input on the x axis of the total stick area
    double rAxisX = rotationXAxis.getAsDouble();
    // Gets position of joystick input on the y axis of the total stick area
    double rAxisY = -rotationYAxis.getAsDouble();

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = (Math.abs(yAxis) < kStickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < kStickDeadband) ? 0 : xAxis;
    rAxisX = (Math.abs(rAxisX) < kAzimuthStickDeadband) ? 0 : rAxisX;
    rAxisY = (Math.abs(rAxisY) < kAzimuthStickDeadband) ? 0 : rAxisY;

    translation = new Translation2d(yAxis, xAxis).times(kMaxSpeed);
    if (rAxisX == 0 && rAxisY == 0) {
      swerveSubsystem.drive(translation, 0, fieldRelative, openLoop);
      return;
    }

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees
    double azimuthAngle = (Math.atan2(rAxisY, rAxisX) * 180 / Math.PI) - 90;

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error
    double rotationPIDOutput =
        azimuthController.calculate(swerveSubsystem.getYaw().getDegrees(), azimuthAngle);

    translation = new Translation2d(yAxis, xAxis).times(kMaxSpeed);
    rotationPIDOutput =
        MathUtil.clamp(rotationPIDOutput, -kMaxAngularVelocity, kMaxAngularVelocity);

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
  }
}
