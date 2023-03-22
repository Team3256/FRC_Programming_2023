// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import static frc.robot.mole.MoleConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.mole.Mole;
import frc.robot.mole.Mole.MolePreset;

public class SetMoleAngle extends ProfiledPIDCommand {
  private Mole moleSubsystem;
  private MolePreset molePreset;
  private Rotation2d angleRotation2d;

  public SetMoleAngle(Mole moleSubsystem, Rotation2d angleRotation2d) {
    super(
        new ProfiledPIDController(kP, kI, kD, kProfileContraints),
        moleSubsystem::getMolePositionRads,
        MathUtil.clamp(
            angleRotation2d.getRadians(),
            kMinConstraintAngle.getRadians(),
            kMaxConstraintAngle.getRadians()),
        (output, setpoint) ->
            moleSubsystem.setInputVoltage(
                output + moleSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)),
        moleSubsystem);

    getController().setTolerance(kMolePivotAngleTolerance.getRadians());

    this.angleRotation2d = angleRotation2d;
    this.moleSubsystem = moleSubsystem;
    addRequirements(moleSubsystem);
  }

  public SetMoleAngle(Mole moleSubsystem, MolePreset molePreset) {
    this(moleSubsystem, molePreset.rotation);
    this.molePreset = molePreset;
  }

  @Override
  public void initialize() {
    super.initialize();
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " started (preset: "
              + molePreset
              + ", rotation: "
              + angleRotation2d.getDegrees()
              + " deg)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    moleSubsystem.off();
    if (Constants.kDebugEnabled) {
      System.out.println(
          this.getName()
              + " ended (preset: "
              + molePreset
              + ", rotation: "
              + angleRotation2d.getDegrees()
              + " deg)");
    }
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
