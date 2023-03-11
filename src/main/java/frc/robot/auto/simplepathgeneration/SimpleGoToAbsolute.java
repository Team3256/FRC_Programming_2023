package frc.robot.auto.simplepathgeneration;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathConstraints;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kDynamicPathGenerationDebug;
import static frc.robot.Constants.trajectoryViewer;
import static frc.robot.Constants.waypointViewer;


public class SimpleGoToAbsolute {
    static Command run(SwerveDrive swerveDrive, Pose2d sink){
        Pose2d src = swerveDrive.getPose();
        Rotation2d heading = sink.minus(src).getTranslation().getAngle();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            kDynamicPathConstraints,
            new PathPoint(src.getTranslation(),heading,src.getRotation()),
            new PathPoint(sink.getTranslation(),heading,sink.getRotation()));

        // send trajectory to networktables for logging
        if (kDynamicPathGenerationDebug){
        trajectoryViewer.getObject("DynamicTrajectory").setTrajectory(traj);
        waypointViewer.getObject("Src").setPose(traj.getInitialHolonomicPose());
        waypointViewer.getObject("Sink").setPose(traj.getEndState().poseMeters);
        }

        // create command that runs trajectory
        AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
        Command trajCommand =
          autoBuilder.createPathPlannerCommand(traj, false, false)
        return trajCommand;
    }
}