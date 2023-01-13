package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

	private final TalonFX intakeMotor;

	public Intake() {
		intakeMotor = new TalonFX(21);
		off();
	}

	public void forward(){
		System.out.println("Intake forward");
		intakeMotor.set(ControlMode.PercentOutput, 1);
	}

	public void backward(){
		System.out.println("Intake backward");
		intakeMotor.set(ControlMode.PercentOutput, 1);
	}

	public void off(){
		System.out.println("Intake off");
		intakeMotor.neutralOutput();
	}
}