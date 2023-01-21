package frc.robot.swerve.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BotConstants;
import frc.robot.swerve.SwerveDrive;

public class X extends CommandBase{
	private SwerveDrive swerveDrive;

	public X(SwerveDrive swerveDrive){
		addRequirements(swerveDrive);
		this.swerveDrive=swerveDrive;
	}

	@Override
	public void initialize() {
		//calculate the universal X formation inward angle
		double inwardAngle = Math.tan(BotConstants.length/BotConstants.width);
		//convert inward angle into specific angles for each swerve module
		double[] modAngle = {inwardAngle,180-inwardAngle,inwardAngle,180-inwardAngle};

		//set each swerve module to modAngle and stop movement
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i=0;i<4;i++) states[i] = new SwerveModuleState(0,new Rotation2d(modAngle[i]));
		swerveDrive.setModuleStates(states);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
