// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.helpers.TimedBoolean;
import frc.robot.swerve.SwerveDrive;

public class AutoBalance extends PIDCommand {

  public AutoBalance(SwerveDrive swerveDrive) {
    super(
        new PIDController(kAutoBalanceP, kAutoBalanceI, kAutoBalanceD),
        swerveDrive::getAutoBalanceOffset,
        () -> 0,
        (output) -> swerveDrive.drive(new Translation2d(output, 0), 0, true, true),
        swerveDrive);
    getController().setTolerance(kAutoBalanceTolerance.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return new TimedBoolean(() -> getController().atSetpoint(), 2).hasBeenTrueForThreshold();
  }
}
