package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;


public class Intake extends SubsystemBase {

	private final TalonFX intakeMotor;

	public Intake() {
		intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
		off();
		System.out.println("Intake initialized");
	}

	public void forward(){
		System.out.println("Intake forward");
		intakeMotor.set(ControlMode.PercentOutput, INTAKE_FORWARD_SPEED);
	}

	public void backward(){
		System.out.println("Intake backward");
		intakeMotor.set(ControlMode.PercentOutput, INTAKE_BACKWARD_SPEED);
	}

	public void off(){
		System.out.println("Intake off");
		intakeMotor.neutralOutput();
	}
}