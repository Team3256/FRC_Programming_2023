// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;

// TODO: Use our own teleop command
public class TeleopSwerve extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveDrive;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  /** Driver control */
  public TeleopSwerve(
      SwerveDrive swerveDrive,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void initialize() {
    swerveDrive.setAngleMotorsNeutralMode(angleNeutralMode);
    swerveDrive.setDriveMotorsNeutralMode(driveNeutralMode);
  }

  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    double rAxis = -controller.getRawAxis(rotationAxis);

    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

    translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
    rotation = rAxis * maxAngularVelocity * 0.25;
    swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
  }
}
