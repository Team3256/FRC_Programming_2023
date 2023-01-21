package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorSetHeight extends PIDCommand{
	Elevator elevator;
	public ElevatorSetHeight(Elevator elevator, double tar) {
		super(
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD),
        // Close loop on heading
        elevator::getPosition,
        // Set reference to targt
        tar,
        // Pipe output to turn robot
        output -> elevator.setSpeed(output),
        // Require the drive
       	elevator);
		this.elevator=elevator;

		getController().setTolerance(ElevatorConstants.kTolerancePosition, ElevatorConstants.kToleranceRate);
	}

	@Override
	public boolean isFinished() {
		return getController().atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		elevator.off();
	}
}
