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
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveIO;
import frc.robot.swerve.SwerveModuleIO;
import frc.robot.swerve.commands.TeleopSwerve;

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

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton intake =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton outtake =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final SwerveDrive swerveDrive;
  private final Intake intakeSubsystem = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    // @suppress-warnings
    switch (Constants.currentMode) {
      case REAL:
        swerveDrive = new SwerveDrive(new SwerveModuleIO());
        break;

      default: // Replay robot - disable IO impl.
        swerveDrive = new SwerveDrive(new SwerveIO() {});
    }

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

    // intake buttons for testing
    intake.whileTrue(new IntakeForward(intakeSubsystem));
    outtake.whileTrue(new Outtake(intakeSubsystem));
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
