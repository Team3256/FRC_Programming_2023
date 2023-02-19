package frc.robot.auto.dynamicpathgeneration;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.swerve.SwerveDrive;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.kBlueImportantLocations;

public class DynamicPathFollower{
	static void run(SwerveDrive swerveDrive){
		//get src, sink
		Pose2d src = swerveDrive.getPose();
		int locationId =(int)SmartDashboard.getNumber("locationId",-1);
		if (locationId==-1){
			System.out.println("locationId entered was invalid.");
			return;
		}
		Pose2d sink = kBlueImportantLocations[locationId];
		//get traj
		DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
		PathPlannerTrajectory trajectory = generator.getTrajectory();
		//create command that runs traj
		AutoBuilder autoBuilder = new AutoBuilder(swerveDrive);
		Command toRun = autoBuilder.createPathPlannerCommand(trajectory,false);
		//run command
		CommandScheduler.getInstance().schedule(toRun);
	}
}
