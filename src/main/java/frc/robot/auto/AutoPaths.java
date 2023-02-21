package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.arm.Arm;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoChooser;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.swerve.SwerveDrive;

import static frc.robot.auto.AutoConstants.*;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoPaths {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;

  public AutoPaths(SwerveDrive swerveSubsystem, Intake intakeSubsystem, Elevator elevatorSubsystem, Arm armSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public void sendCommandsToChooser() {
    AutoChooser.createSinglePath("Do Nothing", new InstantCommand());

    if (swerveSubsystem == null)
      return;

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem);
    ArrayList<Command> testAuto = autoBuilder.createPaths("OH2engageSS1", kDefaultPathConstraints);
    AutoChooser.createIncrementalPaths(
        new ArrayList<String>(Arrays.asList("Top 2 Piece Auto", "Top 2 Piece Auto Engage")), testAuto);
  }

  public Command getSelectedPath() {
    return AutoChooser.getCommand();
  }
}
