// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.mole.commands;

import static frc.robot.mole.Mole.MolePosition.CUBE_GROUND;
import static frc.robot.mole.MoleConstants.kMolePivotAngleTolerance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessBlinkingPattern;
import frc.robot.mole.Mole;

public class MoleIntakeCube extends CommandBase {

  private final Mole moleSubsystem;
  private final Rotation2d desiredMoleAngle = CUBE_GROUND.rotation;

  private LED ledSubsystem;

  public MoleIntakeCube(Mole moleSubsystem) {
    this.moleSubsystem = moleSubsystem;
    addRequirements(moleSubsystem);
  }

  public MoleIntakeCube(Mole moleSubsystem, LED ledSubsystem) {
    this.moleSubsystem = moleSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(moleSubsystem);
  }

  @Override
  public void initialize() {
    moleSubsystem.setPivotPosition(CUBE_GROUND.rotation.getDegrees());
  }

  @Override
  public void execute() {
    if (Math.abs(moleSubsystem.getMolePivotPositionRadians() - desiredMoleAngle.getRadians())
        < kMolePivotAngleTolerance.getRadians()) moleSubsystem.intakeCube();
  }

  @Override
  public void end(boolean interrupted) {
    moleSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern())
          .withTimeout(3)
          .schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return moleSubsystem.isCurrentSpiking();
  }
}
