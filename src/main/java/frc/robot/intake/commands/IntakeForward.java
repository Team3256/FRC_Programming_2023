package frc.robot.intake.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class IntakeForward extends CommandBase {

	private final Intake intake;

	public IntakeForward(Intake subsystem) {
		intake = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		intake.forward();
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