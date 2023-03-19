// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import static frc.robot.mole.MoleConstants.kMolePivotAngleTolerance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;

public class MoleShootCube extends CommandBase {

  Mole moleSubsytem;
  Rotation2d desiredMoleAngle;

  public MoleShootCube(Mole moleSubsystem, Rotation2d desiredMoleAngle) {
    this.moleSubsytem = moleSubsystem;
    this.desiredMoleAngle = desiredMoleAngle;
    addRequirements(moleSubsystem);
  }

  public MoleShootCube(Mole moleSubsystem, Mole.MolePosition desiredMoleAngle) {
    this(moleSubsystem, desiredMoleAngle.rotation);
  }

  @Override
  public void initialize() {
    moleSubsytem.setPivotPosition(desiredMoleAngle.getDegrees());
  }

  @Override
  public void execute() {
    if (Math.abs(moleSubsytem.getMolePivotPositionRadians() - desiredMoleAngle.getRadians())
        < kMolePivotAngleTolerance.getRadians()) moleSubsytem.outtakeCube();
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
