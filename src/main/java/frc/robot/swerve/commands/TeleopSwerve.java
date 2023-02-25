// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.elevator.Elevator;
import frc.robot.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;
  private Elevator elevatorSubsystem;
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

  public TeleopSwerve(
      SwerveDrive swerveSubsystem,
      Elevator elevatorSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this(swerveSubsystem, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void initialize() {
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

    translation = new Translation2d(Math.pow(yAxis, 3), Math.pow(xAxis, 3)).times(kMaxSpeed);
    rotation = rAxis * kMaxAngularVelocity;

    if (elevatorSubsystem != null)
      swerveSubsystem.drive(
          translation, rotation, fieldRelative, openLoop, elevatorSubsystem.getElevatorPosition());
    else swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
  }
}
