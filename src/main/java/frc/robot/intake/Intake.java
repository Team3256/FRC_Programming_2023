// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import static frc.robot.Constants.ShuffleboardConstants.kDriverTabName;
import static frc.robot.intake.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.logging.Loggable;

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
  public void logInit() {
    getLayout(kDriverTabName).add("1", this);
    getLayout(kDriverTabName).add("2", new IntakeCube(this));
    getLayout(kDriverTabName).add("3", new IntakeCone(this));
    getLayout(kDriverTabName).add("4", intakeMotor);
  }

  @Override
  public void logPeriodic() {}

  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab).getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4);
  }

  public boolean test() {
    System.out.println("Testing intake CAN:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake CAN connected: " + result);
    SmartDashboard.putBoolean("Intake CAN connected", result);
    return result;
  }
}
