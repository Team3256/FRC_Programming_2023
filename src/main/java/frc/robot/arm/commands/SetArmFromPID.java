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
import frc.robot.arm.ArmConstants;
import frc.robot.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class SetArmFromPID extends PIDCommand {
  DoubleSupplier velocity;
  ArmSubsystem armSubsystem;
  boolean movingSetpoint = false;

  // TODO: Use ArmK
  public SetArmFromPID(ArmSubsystem armSubsystem) {
    super(
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
        armSubsystem::getAngle,
        SmartDashboard.getNumber("Arm Angle", 0),
        voltage ->
            armSubsystem.setInputVoltage(
                voltage
                    + SmartDashboard.getNumber("Arm Angle", 0)
                        * SmartDashboard.getNumber("Arm KFF", ArmConstants.kFF)),
        armSubsystem);

    this.velocity = () -> SmartDashboard.getNumber("Arm Angle", 0);
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  public SetArmFromPID(ArmSubsystem armSubsystem, double velocity) {
    super(
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
        armSubsystem::getAngle,
        velocity,
        voltage -> armSubsystem.setInputVoltage(voltage + velocity * ArmConstants.kFF),
        armSubsystem);

    this.velocity = () -> velocity;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putData(getController());
    SmartDashboard.putNumber("Arm Angle", 0);
    SmartDashboard.putNumber("Arm KFF", ArmConstants.kFF);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setInputVoltage(0);
  }

  @Override
  public void execute() {
    super.execute();
    if (movingSetpoint) {
      SmartDashboard.putNumber("Arm Angle", velocity.getAsDouble());
    }
  }
}
