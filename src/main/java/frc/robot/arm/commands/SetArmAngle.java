// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;

public class SetArmAngle extends ProfiledPIDCommand {
  private Arm armSubsystem;
  private Rotation2d angleRotation2d;
  private ArmPosition armPosition;

  public SetArmAngle(Arm armSubsystem, Rotation2d angleRotation2d) {
    super(
        new ProfiledPIDController(kP, kI, kD, kArmContraints),
        armSubsystem::getArmPositionRads,
        angleRotation2d.getRadians(),
        (output, setpoint) ->
            armSubsystem.setInputVoltage(
                output + armSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)),
        armSubsystem);

    getController()
        .setTolerance(kArmToleranceAngle.getRadians(), kArmToleranceAngularVelocity.getRadians());
    addRequirements(armSubsystem);
    this.angleRotation2d = angleRotation2d;
    this.armSubsystem = armSubsystem;
  }

  public SetArmAngle(Arm armSubsystem, ArmPosition armPosition) {
    this(armSubsystem, armPosition.rotation);
    this.armPosition = armPosition;
  }

  @Override
  public void initialize() {
    super.initialize();
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " started (position: "
              + this.armPosition
              + ", rotation: "
              + angleRotation2d.getDegrees()
              + " deg)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    armSubsystem.off();
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " finished (position: "
              + this.armPosition
              + ", rotation: "
              + angleRotation2d.getDegrees()
              + " deg)");
    }
  }
}
