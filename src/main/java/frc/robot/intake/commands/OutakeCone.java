
package frc.robot.intake.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessPattern;

public class OutakeCone extends CommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;
  private WPI_TalonFX intakeMotor;

  public OutakeCone(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  public OutakeCone(Intake intakeSubsystem, LED ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.intakeCube();
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
    return intakeMotor.getSelectedSensorPosition() > 1000;
  }
}
