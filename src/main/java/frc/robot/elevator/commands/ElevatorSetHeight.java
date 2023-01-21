package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorSetHeight extends PIDCommand{
	Elevator elevatorSubsystem;
	public ElevatorSetHeight(Elevator elevatorSubsystem, double tar) {
		super(
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD),
        elevatorSubsystem::getPosition,
        tar,
        output -> elevatorSubsystem.setSpeed(output),
       	elevatorSubsystem);
		this.elevatorSubsystem=elevatorSubsystem;
		getController().setTolerance(ElevatorConstants.kTolerancePosition, ElevatorConstants.kToleranceRate);
	}

	@Override
	public boolean isFinished() {
		return getController().atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		elevatorSubsystem.off();
	}
}
