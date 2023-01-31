// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveLimited;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final boolean fieldRelative = true;
  private final boolean openLoop = true;

  private SwerveDrive swerveDrive;
  private Intake intakeSubsystem;

  public RobotContainer() {
    if (INTAKE) configureIntake();
    if (SWERVE) configureSwerve();
    if (ELEVATOR) configureElevator();
  }

  private void configureIntake() {
    this.intakeSubsystem = new Intake();

    driver.leftBumper().whileTrue(new IntakeCube(intakeSubsystem));
    driver.leftTrigger().whileTrue(new IntakeCone(intakeSubsystem));
  }

  private void configureSwerve() {
    this.swerveDrive = new SwerveDrive();

    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            () -> driver.getRightY(),
            () -> driver.getRightX(),
            () -> driver.getLeftX(),
            fieldRelative,
            openLoop));

    driver.a().onTrue(new InstantCommand(swerveDrive::zeroGyro));
    driver
        .b()
        .toggleOnTrue(
            new TeleopSwerveLimited(
                swerveDrive,
                () -> driver.getRightY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                fieldRelative,
                openLoop));
  }

  public void configureElevator() {}

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}
