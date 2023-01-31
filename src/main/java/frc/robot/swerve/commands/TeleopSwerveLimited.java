// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

// TODO: Use our own teleop command
public class TeleopSwerveLimited extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveDrive;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier rotationAxis;

  /** Driver control */
  public TeleopSwerveLimited(
      SwerveDrive swerveDrive,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {

    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void execute() {
    double yAxis = -translationAxis.getAsDouble() * kSensitivityScale;
    double xAxis = -strafeAxis.getAsDouble() * kSensitivityScale;
    double rAxis = -rotationAxis.getAsDouble() * kSensitivityScale;

    /* No deadbands since sensitivity is so low */

    translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
    rotation = rAxis * maxAngularVelocity;
    swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
  }
}
