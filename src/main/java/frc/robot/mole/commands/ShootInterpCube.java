// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import static frc.robot.Constants.FieldConstants.Grids.kLowTranslations;
import static frc.robot.Constants.FieldConstants.Grids.kLowTranslationsRed;
import static frc.robot.Constants.FieldConstants.kFieldLength;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;

public class ShootInterpCube extends CommandBase {

  Mole moleSubsytem;
  Pose2d odomCurrentPosition;

  public ShootInterpCube(Mole moleSubsystem, Pose2d odomCurrentPosition) {
    this.moleSubsytem = moleSubsystem;
    this.odomCurrentPosition = odomCurrentPosition;
    addRequirements(moleSubsystem);
  }

  @Override
  public void execute() {
    double minDistanceFromGridLocation = kFieldLength;

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      for (int i = 0; i < kLowTranslations.length; ++i) {
        minDistanceFromGridLocation =
            Math.min(
                minDistanceFromGridLocation,
                odomCurrentPosition.getTranslation().getDistance(kLowTranslations[i]));
      }
    } else {
      for (int i = 0; i < kLowTranslationsRed.length; ++i) {
        minDistanceFromGridLocation =
            Math.min(
                minDistanceFromGridLocation,
                odomCurrentPosition.getTranslation().getDistance(kLowTranslations[i]));
      }
    }
    moleSubsytem.shootCube(
        moleSubsytem.getMoleShooterRPMFromDistanceInterpolation(minDistanceFromGridLocation));
  }

  @Override
  public void end(boolean interrupted) {
    moleSubsytem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
