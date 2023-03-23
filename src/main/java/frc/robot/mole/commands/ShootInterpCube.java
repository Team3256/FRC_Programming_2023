// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.limelight.Limelight;
import frc.robot.mole.Mole;

public class ShootInterpCube extends CommandBase {

  Mole moleSubsytem;

  public ShootInterpCube(Mole moleSubsystem) {
    this.moleSubsytem = moleSubsystem;
    addRequirements(moleSubsystem);
  }

  @Override
  public void execute() {
    if (Limelight.hasValidTargets(
        Constants.VisionConstants.FrontConstants.kLimelightNetworkTablesName)) {
      double[] botPoseTargetSpace =
          Limelight.getTargetPose_RobotSpace(
              Constants.VisionConstants.FrontConstants.kLimelightNetworkTablesName);
      double botPoseTargetSpaceVerticalDistance = botPoseTargetSpace[2];
      moleSubsytem.shootCube(
          moleSubsytem.getMoleShooterRPMFromDistanceInterpolation(
              botPoseTargetSpaceVerticalDistance));
    }
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
