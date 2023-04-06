// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.kOuttakeRotations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class OuttakeCone extends CommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;

  public OuttakeCone(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  public OuttakeCone(Intake intakeSubsystem, LED ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Started Outtake Cone");
    intakeSubsystem.zeroEncoder();
    intakeSubsystem.outtakeCone();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.off();
    System.out.println("Ended Outtake Cone");
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPattern()).withTimeout(1).schedule();
    }
  }

  // TODO: Make outtake finish after a specified amount of time instead of distance
  @Override
  public boolean isFinished() {
    return Math.abs(intakeSubsystem.getIntakeRevolutions()) > kOuttakeRotations;
  }
}
