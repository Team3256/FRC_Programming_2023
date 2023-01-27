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
import frc.robot.drivers.Loggable;
import frc.robot.intake.commands.IntakeForward;
import frc.robot.intake.commands.Outtake;

public class Intake extends SubsystemBase implements Loggable {

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

  @Override
  public void startLog() {
    SmartDashboard.putData(this);
    SmartDashboard.putData("Intake Forward", new IntakeForward(this));
    SmartDashboard.putData("Outtake", new Outtake(this));
  }

  @Override
  public void periodicLog() {
    SmartDashboard.putNumber("Intake Motor Voltage", intakeMotor.getBusVoltage());
  }
}
