// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.led.LEDConstants.kSuccess;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;

public class IntakeCube extends DebugCommandBase {
  private Intake intakeSubsystem;
  private TimedBoolean isCurrentSpiking;
  private LED ledSubsystem;

  public IntakeCube(Intake intakeSubsystem) {
    this(intakeSubsystem, 0.04);
  }

  public IntakeCube(Intake intakeSubsystem, LED ledSubsystem) {
    this(intakeSubsystem);
    this.ledSubsystem = ledSubsystem;
  }

  public IntakeCube(Intake intakeSubsystem, double triggerTime) {
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentSpiking = new TimedBoolean(intakeSubsystem::isCurrentSpiking, triggerTime);
    this.addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    isCurrentSpiking.initialize();
    intakeSubsystem.configIntakeCurrentLimit();
    intakeSubsystem.intakeCube();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new SetAllBlink(ledSubsystem, kSuccess).withTimeout(1).schedule();
    }
    if (!interrupted) {
      new LatchGamePiece(intakeSubsystem, false).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    isCurrentSpiking.update();
    return isCurrentSpiking.hasBeenTrueForThreshold();
  }
}
