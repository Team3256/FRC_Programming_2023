// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import static frc.robot.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;

public class Intake extends SubsystemBase implements CANTestable {
  private WPI_TalonFX intakeMotor;

  public Intake() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    off();
    System.out.println("Intake initialized");
  }

  private void configureRealHardware() {
    intakeMotor = TalonFXFactory.createDefaultTalon(kIntakeCANDevice);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  private void configureSimHardware() {
    intakeMotor = new WPI_TalonFX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getIntakeSpeed() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void intakeCone() {
    System.out.println("Intake cone");
    intakeMotor.set(ControlMode.PercentOutput, kIntakeConeSpeed);
  }

  public void intakeCube() {
    System.out.println("Intake cube");
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed);
  }

  public boolean isCurrentSpiking() {
    return intakeMotor.getSupplyCurrent() >= kIntakeCurrentSpikingThreshold;
  }

  public void off() {
    System.out.println("Intake off");
    intakeMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake current", intakeMotor.getSupplyCurrent());
  }

  public boolean CANTest() {
    System.out.println("Testing intake CAN:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake CAN connected: " + result);
    SmartDashboard.putBoolean("Intake CAN connected", result);
    return result;
  }
}
