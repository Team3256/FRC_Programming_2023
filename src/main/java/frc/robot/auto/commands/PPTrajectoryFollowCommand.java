package frc.robot.auto.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
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

import static frc.robot.Constants.AutoConstants.AUTO_DEBUG;
import static frc.robot.Constants.AutoConstants.TRAJECTORY_DURATION_FACTOR;


public class PPTrajectoryFollowCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final SwerveDriveController controller;
    private final SwerveDrive driveSubsystem;
    private final double trajectoryDuration;
    private Pose2d startPose;
    private AutoCommandRunner autoCommandRunner;

    public PPTrajectoryFollowCommand(
            PathPlannerTrajectory trajectory,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            SwerveDrive driveSubsystem) {


        this.trajectory = trajectory;
        this.trajectoryDuration = trajectory.getTotalTimeSeconds();
        this.controller = new SwerveDriveController(
                xController,
                yController,
                thetaController
        );

        this.driveSubsystem = driveSubsystem;
        PathPlannerTrajectory.PathPlannerState start = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(0.0);
        Rotation2d rotation = start.holonomicRotation;
        Translation2d translation = start.poseMeters.getTranslation();
        this.startPose = new Pose2d(translation, rotation);

        addRequirements(driveSubsystem);
    }

    public PPTrajectoryFollowCommand(
            PathPlannerTrajectory trajectory,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Pose2d startPose,
            SwerveDrive driveSubsystem) {

        this.trajectory = trajectory;
        this.trajectoryDuration = trajectory.getTotalTimeSeconds();
        this.controller = new SwerveDriveController(
                xController,
                yController,
                thetaController
        );

        this.driveSubsystem = driveSubsystem;
        this.startPose = startPose;
        addRequirements(driveSubsystem);
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
        if (AUTO_DEBUG) {
//            driveSubsystem.setTrajectory(trajectory);
        }
        if (this.startPose != null) { // use existing pose for more accuracy if not first path
//            driveSubsystem.resetOdometry(this.startPose);
        }



        this.controller.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double now = timer.get();
        now = now >= trajectoryDuration ? trajectoryDuration - 0.01 : now; // if overtime, dont error sampling

        PathPlannerTrajectory.PathPlannerState desired = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(now);
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d desiredPose = desired.poseMeters;
        double desiredLinearVelocity = desired.velocityMetersPerSecond;

        Rotation2d desiredRotation = desired.holonomicRotation;

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
            SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));
        }

        if (autoCommandRunner != null) {
            autoCommandRunner.execute(desiredPose);
        }

        driveSubsystem.drive(controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation));
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectoryDuration * TRAJECTORY_DURATION_FACTOR;
    } // give a little more time to be in the right place

    @Override
    public void end(boolean interrupted){
        if (autoCommandRunner != null) {
            autoCommandRunner.end();
        }
        driveSubsystem.drive(new ChassisSpeeds());
    }
}

