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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.ArmConstants;

public class SetArmAngle extends ProfiledPIDCommand {
  private Arm armSubsystem;
  private Rotation2d angleRotation2d;
  private ArmPosition armPosition;
  private boolean shouldEnd;

  /**
   * Constructor for setting arm to arbitrary angle in radians
   *
   * @param armSubsystem
   * @param angleRotation2d
   * @param shouldEnd
   */
  public SetArmAngle(Arm armSubsystem, Rotation2d angleRotation2d, boolean shouldEnd) {
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
    this.shouldEnd = shouldEnd;
    addRequirements(armSubsystem);
  }

  /**
   * Constructor for setting the arm to a Rotation2d specified in the preferences hash map
   *
   * @param armSubsystem
   * @param armPosition
   * @param shouldEnd
   */
  public SetArmAngle(Arm armSubsystem, ArmPosition armPosition, boolean shouldEnd) {
    this(armSubsystem, armSubsystem.getPreferencesSetpoint(armPosition), shouldEnd);
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

  @Override
  public boolean isFinished() {
    return getController().atGoal() && shouldEnd;
  }
}
