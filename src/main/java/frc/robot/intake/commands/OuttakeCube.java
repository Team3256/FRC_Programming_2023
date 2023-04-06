// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.kOuttakeRotations;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class OuttakeCube extends CommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;
  private WPI_TalonFX intakeMotor;

  public OuttakeCube(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  public OuttakeCube(Intake intakeSubsystem, LED ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Started Outtake Cube");
    intakeSubsystem.zeroEncoder();
    intakeSubsystem.outtakeCube();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.off();
    if (!interrupted && ledSubsystem != null) {
      new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPattern()).withTimeout(1).schedule();
    }
  }

  // TODO: Make outtake finish after a specified amount of time instead of distance
  @Override
  public boolean isFinished() {
    System.out.println("Ended Outtake Cube");
    return Math.abs(intakeSubsystem.getIntakeRevolutions()) > kOuttakeRotations;
  }
}
