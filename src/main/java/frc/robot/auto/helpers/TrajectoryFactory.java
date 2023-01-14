package frc.robot.auto.helpers;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auto.commands.PPTrajectoryFollowCommand;
import frc.robot.auto.commands.TrajectoryFollowCommand;
import frc.robot.swerve.SwerveDrive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
//P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER

import static frc.robot.Constants.AutoConstants.*;



public class TrajectoryFactory {
    public enum Direction {
        X, Y
    }

    private final SwerveDrive drive;

    public TrajectoryFactory(SwerveDrive drive) {
        this.drive = drive;
    }

    public Command createPathPlannerCommand(String path, Pose2d startPose) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        return getCommand(trajectory, startPose);
    }

    public Command createPathPlannerCommand(String path, AutoCommandRunner autoCommandRunner) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        PPTrajectoryFollowCommand command = getCommand(trajectory);
        command.setAutoCommandRunner(autoCommandRunner);
        return command;
    }

    public Command createPathPlannerCommand(String path, AutoCommandRunner autoCommandRunner, boolean firstSegment) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        PPTrajectoryFollowCommand command = getCommand(trajectory);
        command.setFirstSegment(firstSegment);
        command.setAutoCommandRunner(autoCommandRunner);
        return command;
    }

    public Command createPathPlannerCommand(String path) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        return getCommand(trajectory);
    }

    public Command createPathPlannerCommand(String path, double max_vel, double max_accel) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, max_vel, max_accel);
        return getCommand(trajectory);
    }

    public Command createPathPlannerCommand(String path, double max_vel, double max_accel, double thetakp, double thetaki, double thetakd) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, max_vel, max_accel);
        return getCommand(trajectory, thetakp, thetaki, thetakd);
    }

    public Command createPathPlannerCommand(String path, AutoCommandRunner runner, double max_vel, double max_accel, double thetakp, double thetaki, double thetakd) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, max_vel, max_accel);
        PPTrajectoryFollowCommand command = getCommand(trajectory, thetakp, thetaki, thetakd);
        command.setAutoCommandRunner(runner);
        return command;
    }

    public Command createPathPlannerCommand(String path, AutoCommandRunner runner, double max_vel, double max_accel, double thetakp, double thetaki, double thetakd, boolean firstSegment) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, max_vel, max_accel);
        PPTrajectoryFollowCommand command = getCommand(trajectory, thetakp, thetaki, thetakd);
        command.setAutoCommandRunner(runner);
        command.setFirstSegment(firstSegment);
        return command;
    }

    public Command createCommand(String jsonFilePath) {
        Trajectory trajectory = generateTrajectoryFromJSON(jsonFilePath);
        ThetaSupplier thetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(String jsonFilePath, ThetaSupplier thetaSupplier) {
        Trajectory trajectory = generateTrajectoryFromJSON(jsonFilePath);
        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(String jsonFilePath, ThetaSupplier thetaSupplier, Pose2d startPose) {
        Trajectory trajectory = generateTrajectoryFromJSON(jsonFilePath);
        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier, startPose);
    }

    public Command createCommand(double start, double end, Direction direction) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig());

        ThetaSupplier thetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }


    public Command createCommand(double start, double end, Direction direction, ThetaSupplier thetaSupplier, boolean reversed) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig(reversed));

        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    private List<Pose2d> createStraightWaypoints(double start, double end, Direction direction) {
        List<Pose2d> waypoints = new ArrayList<>();
        if (direction == Direction.X) {
            for(double pos = start; pos <= end; pos += ((end-start)/20)) { // create 20 waypoints
                waypoints.add(new Pose2d(pos, 0, new Rotation2d()));
            }
        }
        else if (direction == Direction.Y) {
            for(double pos = start; pos <= end; pos += ((end-start)/20)) { // create 20 waypoints
                waypoints.add(new Pose2d(0, pos, new Rotation2d()));
            }
        }
        return waypoints;
    }

    public Command createCommand(double start, double end, Direction direction, ThetaSupplier thetaSupplier) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig());

        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    private Command getCommand(Trajectory trajectory, ThetaSupplier uniformThetaSupplier) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        return new TrajectoryFollowCommand(
                    trajectory,
                    new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                    new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                    uniformThetaSupplier::rotationSupply,
                    thetaController,
                    drive
                );
    }


    private Command getCommand(Trajectory trajectory, ThetaSupplier uniformThetaSupplier, Pose2d startPose) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        return new TrajectoryFollowCommand(
                trajectory,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                uniformThetaSupplier::rotationSupply,
                thetaController,
                startPose,
                drive
        );
    }

    private PPTrajectoryFollowCommand getCommand(PathPlannerTrajectory trajectory) {
        return getCommand(trajectory, P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER);
    }

    private PPTrajectoryFollowCommand getCommand(PathPlannerTrajectory trajectory, double thetakp, double thetaki, double thetakd) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        thetakp, thetaki, thetakd, THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        return new PPTrajectoryFollowCommand(
                trajectory,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaController,
                drive
        );
    }

    private PPTrajectoryFollowCommand getCommand(PathPlannerTrajectory trajectory, Pose2d startPose) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        return new PPTrajectoryFollowCommand(
                trajectory,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaController,
                startPose,
                drive
        );
    }

    private TrajectoryConfig getDefaultTrajectoryConfig() {
        return new TrajectoryConfig(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)
                .setKinematics(this.drive.getKinematics());
    }

    private TrajectoryConfig getDefaultTrajectoryConfig(boolean reversed) {
        return new TrajectoryConfig(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)
                .setReversed(reversed)
                .setKinematics(this.drive.getKinematics());
    }

    private Trajectory generateTrajectoryFromJSON(String trajectoryFile){
        Trajectory trajectory = new Trajectory();
        String fileExtension;
        try {
            fileExtension = trajectoryFile.split("\\.")[trajectoryFile.split("\\.").length-1];
        } catch (ArrayIndexOutOfBoundsException e) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryFile, e.getStackTrace());
            return new Trajectory();
        }

        switch (fileExtension) {
            case "path": // for PathPlanner files
                String path = trajectoryFile.replaceFirst("\\.(.*)", "");
                trajectory = PathPlanner.loadPath(path, kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
                break;
            case "json": // for wpilib PathWeaver files
                try {
                    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
                    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (IOException ex) {
                    DriverStation.reportError("Unable to open trajectory: " + trajectoryFile, ex.getStackTrace());
                }
                break;
            default:
                trajectory = new Trajectory();
        }
        return trajectory;
    }
}
