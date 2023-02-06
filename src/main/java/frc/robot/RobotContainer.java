// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShuffleboardConstants.driverTab;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.GyroSendable;
import frc.robot.drivers.Loggable;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LEDStrip;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.commands.LEDToggleGamePieceDisplay;
import frc.robot.led.patterns.ColorChaseBluePattern;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveLimited;
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDrive swerveDrive;
  private Intake intakeSubsystem;
  private LEDStrip ledStrip;

  boolean cubePiece = true;

  private final ArrayList<CANTestable> testables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  
    if (kIntakeEnabled) {
      configureIntake();
      testables.add(intakeSubsystem);
      loggables.add(intakeSubsystem);
    }
    if (kSwerveEnabled) {
      configureSwerve();
      testables.add(swerveDrive);
      loggables.add(swerveDrive);
    }
    if (kElevatorEnabled) {
      configureElevator();
    }
    if (kLedStripEnabled) {
      configureLEDStrip();
    }
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
            kFieldRelative,
            kOpenLoop));

    driver
        .rightBumper()
        .whileTrue(
            new TeleopSwerveWithAzimuth(
                swerveDrive,
                () -> driver.getRightY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                kFieldRelative,
                kOpenLoop));

    driver.a().onTrue(new InstantCommand(swerveDrive::zeroGyro));
    driver
        .b()
        .toggleOnTrue(
            new TeleopSwerveLimited(
                swerveDrive,
                () -> driver.getRightY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                kFieldRelative,
                kOpenLoop));
  }

  public void configureElevator() {}

  public void configureLEDStrip() {
    ledStrip = new LEDStrip(0, new int[] {100});
    driver.a().onTrue(new LEDToggleGamePieceDisplay(ledStrip));
    driver.b().onTrue(new LEDSetAllSectionsPattern(ledStrip, new ColorChaseBluePattern()));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void startLog() {
    for (Loggable device : loggables) device.startLog();
    Shuffleboard.getTab(driverTab)
        .add(
            "Joystick", new GyroSendable(() -> Math.atan2(driver.getRightX(), driver.getRightY())));
  }

  public void periodicLog() {
    for (Loggable device : loggables) device.periodicLog();
  }

  public void test() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : testables) result &= subsystem.test();
    System.out.println("CAN fully connected: " + result);
  }
}
