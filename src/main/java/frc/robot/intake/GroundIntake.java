package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.drivers.TalonFXFactory;

import static frc.robot.intake.IntakeConstants.kIntakeCANDevice;
import static frc.robot.intake.IntakeConstants.kIntakeMotorID;

public class GroundIntake {

    private WPI_TalonFX armMotor;
    private WPI_TalonFX armIntake;
        public Intake() {
            if (RobotBase.isReal()) {
                configureGroundIntake();
            } else {
                configureGroundSim();
            }
            off();
            System.out.println("Intake initialized");
        }

    private void configureGroundIntake() {
        intakeMotor = TalonFXFactory.createDefaultTalon(kIntakeCANDevice);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void configureGroundSim() {
        intakeMotor = new WPI_TalonFX(kIntakeMotorID);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

}
