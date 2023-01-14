// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import static frc.robot.Constants.AutoConstants.AUTO_DEBUG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.auto.helpers.SwerveDriveController;
import frc.robot.swerve.SwerveDrive;
import java.util.function.Function;

public class TrajectoryFollowCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final Trajectory trajectory;
  private final SwerveDriveController controller;
  private final Function<Double, Rotation2d> thetaFeeder;
  private final SwerveDrive driveSubsystem;
  private final double trajectoryDuration;
  private final Pose2d startPose;

  public TrajectoryFollowCommand(
      Trajectory trajectory,
      PIDController xController,
      PIDController yController,
      Function<Double, Rotation2d> thetaFeeder,
      ProfiledPIDController thetaController,
      SwerveDrive driveSubsystem) {

    this.trajectory = trajectory;
    this.trajectoryDuration = trajectory.getTotalTimeSeconds();
    this.controller = new SwerveDriveController(xController, yController, thetaController);

    this.driveSubsystem = driveSubsystem;
    this.thetaFeeder = thetaFeeder;

    this.startPose = trajectory.sample(0.0).poseMeters;

    addRequirements(driveSubsystem);
  }

  public TrajectoryFollowCommand(
      Trajectory trajectory,
      PIDController xController,
      PIDController yController,
      Function<Double, Rotation2d> thetaFeeder,
      ProfiledPIDController thetaController,
      Pose2d startPose,
      SwerveDrive driveSubsystem) {

    this.trajectory = trajectory;
    this.trajectoryDuration = trajectory.getTotalTimeSeconds();
    this.controller = new SwerveDriveController(xController, yController, thetaController);
    this.driveSubsystem = driveSubsystem;
    this.thetaFeeder = thetaFeeder;
    this.startPose = startPose;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (AUTO_DEBUG) {
      driveSubsystem.setTrajectory(trajectory);
    }

    this.controller.reset();
    driveSubsystem.resetOdometry(this.startPose);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double now = timer.get();
    now =
        now > trajectoryDuration
            ? trajectoryDuration - 0.01
            : now; // if overtime, dont error sampling

    Trajectory.State desired = trajectory.sample(now);
    Pose2d currentPose = driveSubsystem.getPose();
    Pose2d desiredPose = desired.poseMeters;
    double desiredLinearVelocity = desired.velocityMetersPerSecond;

    // Move to the desired rotation a proportion of the way through the whole trajectory
    Rotation2d desiredRotation = thetaFeeder.apply(now);

    if (Constants.DEBUG) {
      SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
      SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));
    }

    driveSubsystem.drive(
        controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation));
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectoryDuration * 1.05;
  } // give a little more time to be in the right place}

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
  }
}
