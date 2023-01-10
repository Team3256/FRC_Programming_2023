// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.TeleopSwerveFieldOriented;
import frc.robot.swerve.helpers.ControllerUtil;

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
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
  private final XboxController driverController = new XboxController(0);

  /* Subsystems */
  private final SwerveDrive swerveDrive = new SwerveDrive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    Command defaultDriveCommand = new TeleopSwerveFieldOriented(
            swerveDrive,
            () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );
    swerveDrive.setDefaultCommand(defaultDriveCommand);
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
    zeroGyro.onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
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
