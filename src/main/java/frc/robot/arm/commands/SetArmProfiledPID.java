// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class SetArmProfiledPID extends ProfiledPIDCommand {
  DoubleSupplier velocity;
  ArmSubsystem armSubsystem;
  boolean movingSetpoint = false;

  public SetArmProfiledPID(ArmSubsystem armSubsystem) {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD,
            new TrapezoidProfile.Constraints(0, 0)),
        armSubsystem::getAngularVelocityRPM,
        () -> 2000,
        (output, setpoint) -> armSubsystem.setInputVoltage(output),
        armSubsystem);
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putData(getController());
    SmartDashboard.putNumber("Velocity Setpoint", 1200);
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
      SmartDashboard.putNumber("Velocity Setpoint", velocity.getAsDouble());
    }
  }
}
