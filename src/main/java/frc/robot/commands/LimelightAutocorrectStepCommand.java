package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.SwerveDrive;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.LimelightAutoCorrectConstants.*;

public class LimelightAutocorrectStepCommand extends SequentialCommandGroup {
    private WeightedObservedPoints data;
    private int distance;


    /**
     * @param robotDrive drivetrain to move forward
     * @param data datapoints to post to
     * @param distance current actual distance to goal
     * Plot datapoint and go forward
     */
    public LimelightAutocorrectStepCommand(SwerveDrive robotDrive, WeightedObservedPoints data, int distance)
    {
        addCommands(new WaitCommand(1),
                new InstantCommand(this::addPoint),
                new WaitCommand(1),
                goForward(robotDrive));
        this.data=data;
        this.distance=distance;
    }

    private Command goForward(SwerveDrive robotDrive){
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(robotDrive.getKinematics());

        List<Pose2d> waypoints = new ArrayList<>();
        for(int pos = 0; pos <= PACE_SIZE; pos++){
            waypoints.add(new Pose2d(Units.inchesToMeters(pos), 0, new Rotation2d()));
        }
//        List<Translation2d> waypoints = List.of(new Translation2d(Units.inchesToMeters(12), 0));s
        // JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory1 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        waypoints,
                        config);

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory1,
                robotDrive::getPose, // Functional interface to feed supplies
                robotDrive.getKinematics(),


                // Position controllers
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);

        return swerveControllerCommand.andThen(() -> robotDrive.drive(new ChassisSpeeds()));
    }

    private void addPoint(){
        data.add(distance,distance-Limelight.getRawDistanceToTarget());
    }
}
