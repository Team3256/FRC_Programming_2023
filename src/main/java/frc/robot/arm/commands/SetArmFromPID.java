// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class SetArmFromPID extends PIDCommand {
  DoubleSupplier velocity;
  ArmSubsystem armSubsystem;
  boolean movingSetpoint = false;
  // TODO: Use ArmK
  public SetArmFromPID(ArmSubsystem armSubsystem) {
    super(
        new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD),
        armSubsystem::getAngularVelocityRPM,
        SmartDashboard.getNumber("Velocity Setpoint", 1200),
        voltage ->
            armSubsystem.setInputVoltage(
                voltage
                    + SmartDashboard.getNumber("Velocity Setpoint", 1200)
                        * SmartDashboard.getNumber("Arm KFF", ArmConstants.KFF)),
        armSubsystem);

    this.velocity = () -> SmartDashboard.getNumber("Velocity Setpoint", 1200);
    this.armSubsystem = armSubsystem;
  }

  public SetArmFromPID(ArmSubsystem armSubsystem, double velocity) {
    super(
        new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD),
        armSubsystem::getAngularVelocityRPM,
        velocity,
        voltage -> armSubsystem.setInputVoltage(voltage + velocity * ArmConstants.KFF),
        armSubsystem);

    this.velocity = () -> velocity;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    SmartDashboard.putData(getController());
    SmartDashboard.putNumber("Velocity Setpoint", 1200);
    SmartDashboard.putNumber("Arm KFF", ArmConstants.KFF);
  }

  // @Override // ?
  public void end() {
    armSubsystem.setInputVoltage(0);
  }

  @Override
  public void execute() {
    super.execute();
    if (movingSetpoint) {
      SmartDashboard.putNumber("Velocity Setpoint", velocity.getAsDouble());
    }
  }
}
