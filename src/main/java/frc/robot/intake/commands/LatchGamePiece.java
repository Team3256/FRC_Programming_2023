// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class LatchGamePiece extends CommandBase {
  private Intake intakeSubsystem;
  private BooleanSupplier isCurrentPieceCone;

  public LatchGamePiece(Intake intakeSubsystem, BooleanSupplier isCone) {
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentPieceCone = isCone;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    if (isCurrentPieceCone.getAsBoolean()) {
      intakeSubsystem.latchCone();
    } else {
      intakeSubsystem.latchCube();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
