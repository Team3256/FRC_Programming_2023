// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;

public class Intake extends SubsystemBase implements CANTestable{

  private final TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(intakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    off();
    System.out.println("Intake initialized");
  }

  public void forward() {
    System.out.println("Intake forward");
    intakeMotor.set(ControlMode.PercentOutput, kIntakeForwardSpeed);
  }

  public void backward() {
    System.out.println("Intake backward");
    intakeMotor.set(ControlMode.PercentOutput, kOuttakeSpeed);
  }

  public void off() {
    System.out.println("Intake off");
    intakeMotor.neutralOutput();
  }

  public boolean test() {
    System.out.println("Testing intake:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake connected: " + result);
    SmartDashboard.putBoolean("Intake connected",result);
    return result;
  }
}
