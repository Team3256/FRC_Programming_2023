// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class LatchGamePiece extends DebugCommandBase {
  private Intake intakeSubsystem;
  private BooleanSupplier isCurrentPieceCone;

  public LatchGamePiece(Intake intakeSubsystem, BooleanSupplier isCurrentPieceCone) {
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
    addRequirements(intakeSubsystem);
  }

  public LatchGamePiece(Intake intakeSubsystem, boolean isCurrentPieceCone) {
    this(intakeSubsystem, () -> isCurrentPieceCone);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.configureLatchCurrentLimit(true);
  }

  @Override
  public void execute() {
    if (isCurrentPieceCone.getAsBoolean()) {
      intakeSubsystem.latchCone();
    } else {
      intakeSubsystem.latchCube();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.configureLatchCurrentLimit(false);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
