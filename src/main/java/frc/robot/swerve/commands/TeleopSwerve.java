// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends DebugCommandBase {
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier rotationAxis;

  public TeleopSwerve(
      SwerveDrive swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void initialize() {
    super.initialize();
    swerveSubsystem.setAngleMotorsNeutralMode(kAngleNeutralMode);
    swerveSubsystem.setDriveMotorsNeutralMode(kDriveNeutralMode);
  }

  @Override
  public void execute() {
    double yAxis = Math.pow(-translationAxis.getAsDouble(), 3);
    double xAxis = Math.pow(-strafeAxis.getAsDouble(), 3);
    double rAxis = Math.pow(-rotationAxis.getAsDouble(), 3);

    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.kStickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.kStickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.kStickDeadband) ? 0 : rAxis;

    translation = new Translation2d(yAxis, xAxis).times(kMaxSpeed);
    rotation = rAxis * kMaxAngularVelocity;

    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
  }
}
