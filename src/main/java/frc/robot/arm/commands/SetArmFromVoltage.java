// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class SetArmFromVoltage extends CommandBase {
  ArmSubsystem armSubsystem;
  DoubleSupplier voltage;
  boolean usingDashboard;

  public SetArmFromVoltage(ArmSubsystem armSubsystem, DoubleSupplier voltage) {
    this.voltage = voltage;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  public SetArmFromVoltage(ArmSubsystem armSubsystem) {
    this.voltage = () -> SmartDashboard.getNumber("Arm Voltage", 0);
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    if (!usingDashboard) {
      SmartDashboard.putNumber("Arm Voltage", voltage.getAsDouble());
    } else {
      SmartDashboard.putNumber("Arm Voltage", voltage.getAsDouble());
    }
  }

  @Override
  public void execute() {
    armSubsystem.setInputVoltage(voltage.getAsDouble());
    SmartDashboard.putNumber("Arm Voltage", armSubsystem.getAngularVelocityRPM());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setInputVoltage(0);
  }
}
