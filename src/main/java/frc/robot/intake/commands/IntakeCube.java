// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class IntakeCube extends CommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;

  public IntakeCube(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  public IntakeCube(Intake intakeSubsystem, LED ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Started Intake cube");
    intakeSubsystem.intakeCube();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended Intake cube");
    intakeSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPattern()).withTimeout(1).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return intakeSubsystem.isCurrentSpiking();
  }
}
