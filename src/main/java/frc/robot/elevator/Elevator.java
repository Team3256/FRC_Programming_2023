package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
		masterElevatorMotor.setNeutralMode(NeutralMode.Brake);
		System.out.println("Elevator initialized");
		off();
	}

	public void setSpeed(double speed){
		masterElevatorMotor.set(ControlMode.PercentOutput, speed);
		System.out.println("Elevator speed: "+speed);
	}

	public double getPosition(){
		return masterElevatorMotor.getSelectedSensorPosition();
	}

	public void off(){
		masterElevatorMotor.neutralOutput();
		System.out.println("Elevator off");
	}
}