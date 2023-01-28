// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeForward;
import frc.robot.intake.commands.Outtake;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  Joystick driverJoystick = new Joystick(0);
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationXAxis = XboxController.Axis.kRightX.value;
  private final int rotationYAxis = XboxController.Axis.kRightY.value;

  /* Subsystems */
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Intake intakeSubsystem = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            driverJoystick,
            translationAxis,
            strafeAxis,
            rotationXAxis,
            Constants.fieldRelative,
            Constants.openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    driver.a().onTrue(new InstantCommand(swerveDrive::zeroGyro));

    // intake buttons for testing
    driver.leftTrigger().whileTrue(new IntakeForward(intakeSubsystem));
    driver.leftBumper().whileTrue(new Outtake(intakeSubsystem));
    driver
        .rightBumper()
        .whileTrue(
            new TeleopSwerveWithAzimuth(
                swerveDrive,
                driverJoystick,
                translationAxis,
                strafeAxis,
                rotationXAxis,
                rotationYAxis,
                Constants.fieldRelative,
                Constants.openLoop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
