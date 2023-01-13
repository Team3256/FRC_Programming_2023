package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class ElevatorOff extends CommandBase{
	private final Elevator elevator;

	public ElevatorOff(Elevator subsystem) {
		elevator = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		elevator.off();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
