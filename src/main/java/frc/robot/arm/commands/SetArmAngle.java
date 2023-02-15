// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.arm.Arm;

public class SetArmAngle extends ProfiledPIDCommand {
  public SetArmAngle(Arm armSubsystem, Rotation2d angleRotation2d) {
    super(
        new ProfiledPIDController(kP, kI, kD, kArmContraints),
        armSubsystem::getArmPosition,
        MathUtil.clamp(
            angleRotation2d.getRadians(),
            Units.degreesToRadians(kArmAngleConstraint),
            Units.degreesToRadians(180 - kArmAngleConstraint)),
        (output, setpoint) ->
            armSubsystem.setInputVoltage(
                output + armSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)),
        armSubsystem);

    getController()
        .setTolerance(kArmToleranceAngle.getRadians(), kArmToleranceAngularVelocity.getRadians());
  }
}
