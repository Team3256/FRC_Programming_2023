// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.kZeroArmVoltage;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeatureFlags;
import frc.robot.arm.Arm;
import frc.robot.helpers.DebugCommandBase;

public class ZeroArm extends DebugCommandBase {
  private final Arm armSubsystem;

  public ZeroArm(Arm armSubsystem) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    super.initialize();
    armSubsystem.setInputVoltage(kZeroArmVoltage);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    armSubsystem.off();
    if (!interrupted) {
      if (FeatureFlags.kUseRelativeArmEncoder) {
        new WaitCommand(0.05)
            .andThen(new InstantCommand(armSubsystem::zeroArmEncoderElevatorRelative))
            .schedule();
      } else {
        armSubsystem.zeroThroughboreEncoder();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.isMotorCurrentSpiking();
  }
}
