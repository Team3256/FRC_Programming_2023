// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import static frc.robot.auto.AutoConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.auto.helpers.SwerveDriveController;
import frc.robot.swerve.SwerveDrive;

public class PPTrajectoryFollowCommand extends CommandBase {
  private final Timer timer = new Timer();
  private PathPlannerTrajectory trajectory;
  private final SwerveDriveController controller;
  private final SwerveDrive swerveSubsystem;
  private final double trajectoryDuration;
  private boolean useAllianceColor;
  private Pose2d startPose;
  private AutoCommandRunner autoCommandRunner;
  private boolean isFirstSegment;

  public PPTrajectoryFollowCommand(
      PathPlannerTrajectory trajectory,
      PIDController xTranslationController,
      PIDController yTranslationController,
      ProfiledPIDController thetaController,
      SwerveDrive swerveSubsystem) {

    this.trajectory = trajectory;
    this.trajectoryDuration = trajectory.getTotalTimeSeconds();
    this.controller =
        new SwerveDriveController(xTranslationController, yTranslationController, thetaController);

    this.swerveSubsystem = swerveSubsystem;
    PathPlannerTrajectory.PathPlannerState start =
        (PathPlannerTrajectory.PathPlannerState) trajectory.sample(0.0);
    Rotation2d rotation = start.holonomicRotation;
    Translation2d translation = start.poseMeters.getTranslation();
    this.startPose = new Pose2d(translation, rotation);

    addRequirements(swerveSubsystem);
  }

  public PPTrajectoryFollowCommand(
      PathPlannerTrajectory trajectory,
      PIDController xTranslationController,
      PIDController yTranslationController,
      ProfiledPIDController thetaController,
      boolean useAllianceColor,
      boolean isFirstSegment,
      SwerveDrive swerveSubsystem) {
    this(
        trajectory,
        xTranslationController,
        yTranslationController,
        thetaController,
        swerveSubsystem);

    this.useAllianceColor = useAllianceColor;
    this.isFirstSegment = isFirstSegment;
    addRequirements(swerveSubsystem);
  }

  public PPTrajectoryFollowCommand(
      PathPlannerTrajectory trajectory,
      PIDController xTranslationController,
      PIDController yTranslationController,
      ProfiledPIDController thetaController,
      Pose2d startPose,
      SwerveDrive swerveSubsystem) {

    this(
        trajectory,
        xTranslationController,
        yTranslationController,
        thetaController,
        swerveSubsystem);
    this.startPose = startPose;

    addRequirements(swerveSubsystem);
  }

  public void setAutoCommandRunner(AutoCommandRunner commandRunner) {
    this.autoCommandRunner = commandRunner;
  }

  public void setFirstSegment(boolean first) {
    if (!first) {
      this.startPose = null;
    }
  }

  @Override
  public void initialize() {
    if (this.useAllianceColor) {
      trajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
      PathPlannerTrajectory.PathPlannerState start =
          (PathPlannerTrajectory.PathPlannerState) trajectory.sample(0.0);
      Rotation2d rotation = start.holonomicRotation;
      Translation2d translation = start.poseMeters.getTranslation();
      this.startPose = new Pose2d(translation, rotation);
    }
    if (kAutoDebug) {
      swerveSubsystem.setTrajectory(trajectory);
    }
    if (isFirstSegment) { // use existing pose for more accuracy if it is the first path
      swerveSubsystem.setGyro(this.startPose.getRotation().getDegrees());
      swerveSubsystem.resetOdometry(this.startPose);
    }

    this.controller.reset();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double now = MathUtil.clamp(timer.get(), 0, trajectoryDuration);

    PathPlannerTrajectory.PathPlannerState desired =
        (PathPlannerTrajectory.PathPlannerState) trajectory.sample(now);
    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d desiredPose = desired.poseMeters;
    double desiredLinearVelocity = desired.velocityMetersPerSecond;

    Rotation2d desiredRotation = desired.holonomicRotation;

    if (kAutoDebug) {
      SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
      SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));
    }

    if (autoCommandRunner != null) {
      autoCommandRunner.execute(desiredPose);
    }

    swerveSubsystem.drive(
        controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation),
        false);
  }

  @Override
  public boolean isFinished() {
    return isTrajectoryFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (autoCommandRunner != null) {
      // find out what to call the method on
      // .atReference();
      autoCommandRunner.end();
    }
    swerveSubsystem.drive(new ChassisSpeeds(), false);
  }

  public boolean isTrajectoryFinished() {
    double now = timer.get();
    if (now >= trajectoryDuration + kAutoTrajectoryTimeoutSeconds) {
      return true;
    }

    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d relativePose = currentPose.relativeTo(trajectory.getEndState().poseMeters);

    boolean reachedEndTolerance =
        relativePose.getTranslation().getNorm() < kTranslationToleranceMeters
            && Math.abs(relativePose.getRotation().getRadians()) < kRotationTolerance
            && now >= trajectoryDuration;

    return reachedEndTolerance;
  }
}
