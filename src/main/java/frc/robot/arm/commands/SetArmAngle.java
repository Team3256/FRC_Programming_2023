// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorAngleOffset;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.auto.AutoConstants;

public class SetArmAngle extends ProfiledPIDCommand {
  private Arm armSubsystem;
  private double angleRotation;
  private ArmPreset armPreset;

  /**
   * Constructor for setting arm to arbitrary angle in radians. This command is RELATIVE to the
   * Elevator Angle!
   *
   * @param armSubsystem
   * @param angleRotation2d
   */
  public SetArmAngle(Arm armSubsystem, Rotation2d angleRotation2d) {
    super(
        new ProfiledPIDController(
            Preferences.getDouble(ArmPreferencesKeys.kPKey, kArmP),
            Preferences.getDouble(ArmPreferencesKeys.kIKey, kArmI),
            Preferences.getDouble(ArmPreferencesKeys.kDKey, kArmD),
            kArmProfileContraints),
        armSubsystem::getArmPositionGroundRelative,
        MathUtil.clamp(
            angleRotation2d.getRadians() + kElevatorAngleOffset,
            kArmAngleMinConstraint.getRadians() + kElevatorAngleOffset,
            kArmAngleMaxConstraint.getRadians() + kElevatorAngleOffset),
        (output, setpoint) ->
            armSubsystem.setInputVoltage(
                output + armSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)),
        armSubsystem);

    getController()
        .setTolerance(kArmToleranceAngle.getRadians(), kArmToleranceAngularVelocity.getRadians());

    this.angleRotation = angleRotation2d.getRadians();
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
      angleRotation = armSubsystem.getArmSetpoint(armPreset).getRadians();
      getController().setGoal(angleRotation + kElevatorAngleOffset);
    }

    if (AutoConstants.kAutoDebug) {
      System.out.println(
          this.getName()
              + " started (preset: "
              + armPreset
              + ", setpoint rotation elevator relative: "
              + Units.radiansToDegrees(angleRotation)
              + " deg)"
              + ", current arm rotation elevator relative: "
              + Units.radiansToDegrees(armSubsystem.getArmPositionElevatorRelative())
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
              + Units.radiansToDegrees(angleRotation)
              + " deg)");
    }
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
