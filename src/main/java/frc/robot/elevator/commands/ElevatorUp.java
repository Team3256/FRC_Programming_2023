package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class ElevatorUp extends CommandBase{
	private final Elevator elevator;

	public ElevatorUp(Elevator subsystem) {
		elevator = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		elevator.up();
	}

	@Override
	public void execute(){

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
