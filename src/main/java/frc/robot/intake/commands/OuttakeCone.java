// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class OuttakeCone extends CommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;
  private final Timer timer = new Timer();
  private double outtakeTime = 2.5;

  public OuttakeCone(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  public OuttakeCone(Intake intakeSubsystem, double time) {
    this(intakeSubsystem);
    this.outtakeTime = time;
  }

  public OuttakeCone(Intake intakeSubsystem, LED ledSubsystem) {
    this(intakeSubsystem);
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
    intakeSubsystem.outtakeCone();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPattern()).withTimeout(1).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(outtakeTime);
  }
}
