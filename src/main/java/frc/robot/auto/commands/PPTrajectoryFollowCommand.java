// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import static frc.robot.auto.AutoConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.auto.helpers.SwerveDriveController;
import frc.robot.auto.helpers.TrajectoryMirrorer;
import frc.robot.swerve.SwerveDrive;

public class PPTrajectoryFollowCommand extends CommandBase {
  private static Field2d autoVisualization = new Field2d();
  private final Timer timer = new Timer();
  private PathPlannerTrajectory trajectory;
  private final SwerveDriveController controller;
  private final SwerveDrive swerveSubsystem;
  private final double trajectoryDuration;
  private boolean useAllianceColor;
  private Pose2d startPose;
  private AutoCommandRunner autoCommandRunner;
  private Alliance alliance;
  private boolean isFirstSegment;

  static {
    if (kAutoDebug && RobotBase.isSimulation()) {
      SmartDashboard.putData("Auto Visualization", autoVisualization);
    }
  }

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
    isFirstSegment = first;
  }

  @Override
  public void initialize() {
    if (this.useAllianceColor) {
      this.alliance = DriverStation.getAlliance();
    } else {
      this.alliance = Alliance.Blue;
      // No mirroring on blue alliance
    }

    PathPlannerTrajectory.PathPlannerState start =
        TrajectoryMirrorer.mirrorState(
            (PathPlannerTrajectory.PathPlannerState) trajectory.sample(0.0), alliance);
    Rotation2d rotation = start.holonomicRotation;
    Translation2d translation = start.poseMeters.getTranslation();
    this.startPose = new Pose2d(translation, rotation);

    if (kAutoDebug) {
      swerveSubsystem.setTrajectory(trajectory);
      autoVisualization.getObject("traj").setTrajectory(trajectory);
      PathPlannerServer.sendActivePath(trajectory.getStates());
    }
    if (isFirstSegment) { // use existing pose for more accuracy if it is the first path
      swerveSubsystem.resetOdometry(this.startPose);
    }

    this.controller.reset();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double now = timer.get();

    PathPlannerTrajectory.PathPlannerState nonMirroredDesired =
        (PathPlannerTrajectory.PathPlannerState) trajectory.sample(now);

    PathPlannerTrajectory.PathPlannerState desired =
        TrajectoryMirrorer.mirrorState(nonMirroredDesired, alliance);
    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d desiredPose = desired.poseMeters;
    double desiredLinearVelocity = desired.velocityMetersPerSecond;

    Rotation2d desiredRotation = desired.holonomicRotation;

    if (kAutoDebug) {
      SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
      SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));

      if (RobotBase.isSimulation()) {
        autoVisualization.setRobotPose(new Pose2d(desiredPose.getTranslation(), desiredRotation));
        PathPlannerServer.sendPathFollowingData(
            new Pose2d(desiredPose.getTranslation(), desiredRotation),
            new Pose2d(desiredPose.getTranslation(), desiredRotation));
      } else {
        PathPlannerServer.sendPathFollowingData(desiredPose, currentPose);
      }
    }

    if (autoCommandRunner != null) {
      autoCommandRunner.execute(nonMirroredDesired.poseMeters, now);
    }

    swerveSubsystem.drive(
        controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation),
        false);
  }

  @Override
  public boolean isFinished() {
    boolean isTrajectoryFinished = isTrajectoryFinished();
    SmartDashboard.putBoolean("Is traj finished", isTrajectoryFinished);

    return isTrajectoryFinished;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
    ;

    if (autoCommandRunner != null) {
      autoCommandRunner.end();
    }

    if (kAutoDebug) {
      System.out.println(
          "Ended trajectory at time " + timer.get() + " seconds out of " + trajectoryDuration);
      System.out.println("Trajectory was interrupted? " + interrupted);
    }
  }

  public boolean isTrajectoryFinished() {
    double now = timer.get();
    if (now >= trajectoryDuration + kAutoTrajectoryTimeoutSeconds) {
      return true;
    }

    if (kAutoDebug && RobotBase.isSimulation()) {
      return now >= trajectoryDuration;
    }

    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d relativePose = currentPose.relativeTo(trajectory.getEndState().poseMeters);

    boolean reachedEndTolerance =
        relativePose.getTranslation().getNorm() < kTranslationToleranceMeters
            && Math.abs(relativePose.getRotation().getRadians()) < kRotationTolerance;

    return reachedEndTolerance && now >= trajectoryDuration;
  }
}
