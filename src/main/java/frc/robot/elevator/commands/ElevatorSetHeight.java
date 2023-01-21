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
        // Close loop on heading
        elevatorSubsystem::getPosition,
        // Set reference to targt
        tar,
        // Pipe output to turn robot
        output -> elevatorSubsystem.setSpeed(output),
        // Require the drive
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
