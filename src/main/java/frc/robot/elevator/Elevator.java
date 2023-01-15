package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {

	private final TalonFX masterElevatorMotor;
	private final TalonFX followerElevatorMotor;

	public Elevator() {
		masterElevatorMotor = new TalonFX(MASTER_ELEVATOR_MOTOR_ID);
		followerElevatorMotor = new TalonFX(FOLLOWER_ELEVATOR_MOTOR_ID);
		followerElevatorMotor.follow(masterElevatorMotor);
		System.out.println("Elevator initialized");
		off();
	}

	public void move(double distance){
		masterElevatorMotor.set(ControlMode.Position, distance);
		System.out.printf("Elevator up %f\n", distance);
	}

	public void setLowPosition(){
		masterElevatorMotor.set(ControlMode.Position, LOW_POSITION_METERS);
		System.out.println("Elevator Position: LOW");
	}

	public void setMediumPosition(){
		masterElevatorMotor.set(ControlMode.Position, MEDIUM_POSITION_METERS);
		System.out.println("Elevator Position: MEDIUM");
	}

	public void setHighPosition(){
		masterElevatorMotor.set(ControlMode.Position, HIGH_POSITION_METERS);
		System.out.println("Elevator Position: HIGH");
	}

	public void off(){
		masterElevatorMotor.neutralOutput();
		System.out.println("Elevator off");
	}
}