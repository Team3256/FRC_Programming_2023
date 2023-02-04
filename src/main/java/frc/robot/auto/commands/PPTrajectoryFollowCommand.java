// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import static frc.robot.auto.AutoConstants.kAutoDebug;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.auto.helpers.SwerveDriveController;
import frc.robot.swerve.SwerveDrive;

public class PPTrajectoryFollowCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final SwerveDriveController controller;
  private final SwerveDrive swerveSubsystem;
  private final double trajectoryDuration;
  private Pose2d startPose;
  private AutoCommandRunner autoCommandRunner;

  public PPTrajectoryFollowCommand(
      PathPlannerTrajectory trajectory,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      SwerveDrive swerveSubsystem) {

    this.trajectory = trajectory;
    this.trajectoryDuration = trajectory.getTotalTimeSeconds();
    this.controller = new SwerveDriveController(xController, yController, thetaController);

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
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Pose2d startPose,
      SwerveDrive swerveDrive) {

    this(trajectory, xController, yController, thetaController, swerveDrive);
    this.startPose = startPose;

    addRequirements(swerveDrive);
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
    if (kAutoDebug) {
      swerveSubsystem.setTrajectory(trajectory);
    }
    if (this.startPose != null) { // use existing pose for more accuracy if it is the first path
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

    if (Constants.kDebugEnabled) {
      SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
      SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));
    }

    if (autoCommandRunner != null) {
      autoCommandRunner.execute(desiredPose);
    }

    swerveSubsystem.drive(
        controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation));
  }

  // TODO: Fix to give a little more time to be in the right place
  @Override
  public boolean isFinished() {
    return timer.get() >= trajectoryDuration; // * TRAJECTORY_DURATION_FACTOR;
  }

  @Override
  public void end(boolean interrupted) {
    if (autoCommandRunner != null) {
      // find out what to call the method on
      // .atReference();
      autoCommandRunner.end();
    }
    swerveSubsystem.drive(new ChassisSpeeds());
  }

  private Pose2d poseTolerance = new Pose2d();
  private Pose2d poseError;
  private Rotation2d rotationError;

  // calculates pose differences
  public boolean atReference() {

    // ---------------------------------------------
    final Translation2d eTranslate = poseError.getTranslation();
    final Rotation2d eRotate = rotationError;
    final Translation2d tolTranslate = poseTolerance.getTranslation();
    final Rotation2d tolRotate = poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }
}
