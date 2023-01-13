package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class IntakeOff extends CommandBase{
	private final Intake intake;

	public IntakeOff(Intake subsystem) {
		intake = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		intake.off();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
