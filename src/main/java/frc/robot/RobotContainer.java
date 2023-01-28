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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeForward;
import frc.robot.intake.commands.Outtake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDAllSectionSetPattern;
import frc.robot.led.patterns.DrivingPattern;
import frc.robot.led.patterns.SpiritPattern;
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
  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final boolean fieldRelative = true;
  private final boolean openLoop = true;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton intake =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton outtake =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton driving = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton bussin = new JoystickButton(driver, XboxController.Button.kY.value);

  private final JoystickButton sensitivityToggle =
      new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Intake intakeSubsystem = new Intake();
  private final LED LEDSubsystem = new LED(0, 75);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            driver,
            translationAxis,
            strafeAxis,
            rotationAxis,
            fieldRelative,
            openLoop));

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
    zeroGyro.onTrue(new InstantCommand(swerveDrive::zeroGyro));
    sensitivityToggle.toggleOnTrue(
        new TeleopSwerveLimited(
            swerveDrive,
            driver,
            translationAxis,
            strafeAxis,
            rotationAxis,
            fieldRelative,
            openLoop));

    // intake buttons for testing
    intake.whileTrue(new IntakeForward(intakeSubsystem));
    outtake.whileTrue(new Outtake(intakeSubsystem));

    // led buttons for fun
    driving.onTrue(new LEDAllSectionSetPattern(LEDSubsystem, new DrivingPattern()));
    bussin.onTrue(new LEDAllSectionSetPattern(LEDSubsystem, new SpiritPattern()));
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
