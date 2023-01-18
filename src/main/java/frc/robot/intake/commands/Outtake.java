package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class Outtake extends CommandBase{

	private final Intake intake;

	public Outtake(Intake subsystem) {
		intake = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		intake.backward();
	}

	@Override
	public void execute() {

	}

	@Override
	public void end(boolean interrupted) {
		intake.off();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}