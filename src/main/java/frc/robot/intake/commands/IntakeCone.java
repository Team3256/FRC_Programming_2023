// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class IntakeCone extends DebugCommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;
  private TimedBoolean isCurrentSpiking;

  public IntakeCone(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentSpiking = new TimedBoolean(intakeSubsystem::isCurrentSpiking, 1);

    addRequirements(intakeSubsystem);
  }

  public IntakeCone(Intake intakeSubsystem, LED ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.intakeCone();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPattern()).withTimeout(1).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    isCurrentSpiking.update();
    return isCurrentSpiking.hasBeenTrueForThreshold();
  }
}
