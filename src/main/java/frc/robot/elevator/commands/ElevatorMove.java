package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorMove extends CommandBase{
	private final Elevator elevator;
	private final DoubleSupplier translationYSupplier;

	public ElevatorMove(Elevator subsystem, DoubleSupplier translationYSupplier) {
		elevator = subsystem;
		this.translationYSupplier = translationYSupplier;
		addRequirements(subsystem);
	}

	public ElevatorMove(Elevator subsystem) {
		elevator = subsystem;
		this.translationYSupplier = () -> 0;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute(){
		elevator.move(translationYSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		elevator.off();
	}
}
