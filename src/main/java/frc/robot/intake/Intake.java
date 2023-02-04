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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.CanDeviceId;
import frc.robot.drivers.Loggable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;

public class Intake extends SubsystemBase implements Loggable, CANTestable {
  private final WPI_TalonFX intakeMotor;

  public double getIntakeSpeed() {
    return intakeMotor.getMotorOutputPercent();
  }

  public Intake() {
    intakeMotor = new WPI_TalonFX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    off();
    System.out.println("Intake initialized");
  }

  public void intakeCone() {
    System.out.println("Intake cone");
    intakeMotor.set(ControlMode.PercentOutput, kIntakeConeSpeed);
  }

  public void intakeCube() {
    System.out.println("Intake cube");
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed);
  }

  public void off() {
    System.out.println("Intake off");
    intakeMotor.neutralOutput();
  }

  @Override
  public void startLog() {
    SmartDashboard.putData(this);
    SmartDashboard.putData("Intake Forward", new IntakeCube(this));
    SmartDashboard.putData("Outtake", new IntakeCone(this));
  }

  @Override
  public void periodicLog() {
    SmartDashboard.putNumber("Intake Motor Voltage", intakeMotor.getBusVoltage());
  }

  public boolean test() {
    System.out.println("Testing intake CAN:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake CAN connected: " + result);
    SmartDashboard.putBoolean("Intake CAN connected", result);
    return result;
  }
}
