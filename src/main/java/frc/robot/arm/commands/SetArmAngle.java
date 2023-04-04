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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.arm.ArmConstants;
import frc.robot.auto.AutoConstants;

public class SetArmAngle extends ProfiledPIDCommand {
  private Arm armSubsystem;
  private Rotation2d angleRotation2d;
  private ArmPreset armPreset;

  /**
   * Constructor for setting arm to arbitrary angle in radians
   *
   * @param armSubsystem
   * @param angleRotation2d
   */
  public SetArmAngle(Arm armSubsystem, Rotation2d angleRotation2d) {
    super(
        new ProfiledPIDController(
            Preferences.getDouble(ArmPreferencesKeys.kPKey, ArmConstants.kP),
            Preferences.getDouble(ArmPreferencesKeys.kIKey, ArmConstants.kI),
            Preferences.getDouble(ArmPreferencesKeys.kDKey, ArmConstants.kD),
            kArmProfileContraints),
        armSubsystem::getArmPositionRads,
        MathUtil.clamp(
            angleRotation2d.getRadians(),
            kArmAngleMinConstraint.getRadians(),
            kArmAngleMaxConstraint.getRadians()),
        (output, setpoint) ->
            armSubsystem.setInputVoltage(
                output + armSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)),
        armSubsystem);

    getController()
        .setTolerance(kArmToleranceAngle.getRadians(), kArmToleranceAngularVelocity.getRadians());

    this.angleRotation2d = angleRotation2d;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  /**
   * Constructor for setting the arm to a Rotation2d specified in the preferences hash map
   *
   * @param armSubsystem
   * @param armPreset
   */
  public SetArmAngle(Arm armSubsystem, ArmPreset armPreset) {
    this(armSubsystem, armSubsystem.getArmSetpoint(armPreset));
    this.armPreset = armPreset;
  }

  @Override
  public void initialize() {
    super.initialize();

    // update at runtime in case robot prefs changed
    if (armPreset != null) {
      angleRotation2d = armSubsystem.getArmSetpoint(armPreset);
      getController().setGoal(angleRotation2d.getRadians());
    }

    if (AutoConstants.kAutoDebug) {
      System.out.println(
          this.getName()
              + " started (preset: "
              + armPreset
              + ", setpoint rotation: "
              + angleRotation2d.getDegrees()
              + " deg)"
              + ", current arm rotation: "
              + Units.radiansToDegrees(armSubsystem.getArmPositionRads())
              + " deg)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    armSubsystem.off();
    if (AutoConstants.kAutoDebug) {
      System.out.println(
          this.getName()
              + " ended (preset: "
              + armPreset
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
