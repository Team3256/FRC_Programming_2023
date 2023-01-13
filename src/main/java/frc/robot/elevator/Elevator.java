package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {

	private final TalonFX elevatorMotor;

	public Elevator() {
		elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
		System.out.println("Elevator Initialized");
		off();
	}

	public void up(){
		System.out.println("Elevator up");
		elevatorMotor.set(ControlMode.PercentOutput,ELEVATOR_UP_SPEED);
	}

	public void down(){
		elevatorMotor.set(ControlMode.PercentOutput,ELEVATOR_DOWN_SPEED);
		System.out.println("Elevator down");
	}

	public void off(){
		elevatorMotor.neutralOutput();
		System.out.println("Intake off");
	}
}