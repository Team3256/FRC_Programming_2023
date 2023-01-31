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
<<<<<<< feat/subsystem-flags
=======
import frc.robot.drivers.CANTestable;
>>>>>>> main
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.TeleopSwerve;
<<<<<<< feat/subsystem-flags
import frc.robot.swerve.commands.TeleopSwerveLimited;
=======
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;
import java.util.ArrayList;
>>>>>>> main

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< feat/subsystem-flags
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
=======
  /* Controllers */
  Joystick driverJoystick = new Joystick(0);
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationXAxis = XboxController.Axis.kRightX.value;
  private final int rotationYAxis = XboxController.Axis.kRightY.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final boolean fieldRelative = true;
  private final boolean openLoop = true;

  /* Subsystems */
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Intake intakeSubsystem = new Intake();

  /* Lists */
  private final ArrayList<CANTestable> testables = new ArrayList<CANTestable>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // add all testables to an iterable
    testables.add(swerveDrive);
    testables.add(intakeSubsystem);
>>>>>>> main

    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
<<<<<<< feat/subsystem-flags
            () -> driver.getRightY(),
            () -> driver.getRightX(),
            () -> driver.getLeftX(),
            fieldRelative,
            openLoop));
=======
            driverJoystick,
            translationAxis,
            strafeAxis,
            rotationXAxis,
            Constants.fieldRelative,
            Constants.openLoop));
>>>>>>> main

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

<<<<<<< feat/subsystem-flags
  public void configureElevator() {}
=======
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
    driver.leftTrigger().whileTrue(new IntakeCone(intakeSubsystem));
    driver.leftBumper().whileTrue(new IntakeCone(intakeSubsystem));
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
>>>>>>> main

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void test() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : testables) result &= subsystem.test();
    System.out.println("CAN fully connected: " + result);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}
