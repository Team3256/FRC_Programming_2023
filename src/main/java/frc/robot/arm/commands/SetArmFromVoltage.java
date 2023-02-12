// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import java.util.function.DoubleSupplier;

public class SetArmFromVoltage extends CommandBase {
  Arm Arm;
  DoubleSupplier voltage;
  boolean usingDashboard;

  public SetArmFromVoltage(Arm Arm, DoubleSupplier set) {
    this.voltage = set;
    this.Arm = Arm;
    addRequirements(Arm);
  }

  public SetArmFromVoltage(Arm Arm) {
    this.voltage = () -> SmartDashboard.getNumber("Arm Voltage", 0);
    this.Arm = Arm;
    addRequirements(Arm);
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
    Arm.setInputVoltage(voltage.getAsDouble());
    SmartDashboard.putNumber("Arm Voltage", voltage.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Arm.setInputVoltage(0);
  }
}
