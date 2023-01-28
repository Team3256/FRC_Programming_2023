// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;

public class TeleopSwerveWithAzimuth extends CommandBase {

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveDrive;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private PIDController azimuthController;
  private int rotationXAxis;
  private int rotationYAxis;

  /** Driver control */
  public TeleopSwerveWithAzimuth(
      SwerveDrive swerveDrive,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationXAxis,
      int rotationYAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.azimuthController = new PIDController(kAzimuthP, kAzimuthI, kAzimuthD);
    azimuthController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    // Obtains axis values for x and y for translation command

    double rAxisX = controller.getRawAxis(rotationXAxis);
    // Gets position of joystick input on the x axis of the total stick area
    double rAxisY = -controller.getRawAxis(rotationYAxis);
    // Gets position of joystick input on the y axis of the total stick area

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    rAxisX = (Math.abs(rAxisX) < Constants.azimuthStickDeadband) ? 0 : rAxisX;
    rAxisY = (Math.abs(rAxisY) < Constants.azimuthStickDeadband) ? 0 : rAxisY;

    SmartDashboard.putNumber("rAxisX", rAxisX);
    SmartDashboard.putNumber("rAxisY", rAxisY);

    SmartDashboard.putNumber("Rotation Angle", swerveDrive.getYaw().getDegrees());

    translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
    if (rAxisX == 0 && rAxisY == 0) {
      swerveDrive.drive(translation, 0, fieldRelative, openLoop);
      return;
    }

    double azimuthAngle = (Math.atan2(rAxisY, rAxisX) * 180 / Math.PI) - 90;
    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // azimuthController.setGoal(azimuthAngle);
    double rotationPIDOutput =
        azimuthController.calculate(swerveDrive.getYaw().getDegrees(), azimuthAngle);
    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error

    SmartDashboard.putNumber("Setpoint Angle", azimuthAngle);
    SmartDashboard.putNumber("Rotation Velocity", rotationPIDOutput);
    SmartDashboard.putData("Azimuth PID Controller", azimuthController);

    translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -maxAngularVelocity, maxAngularVelocity);
    swerveDrive.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
    // Sets motors to the velocities defined here
  }
}
