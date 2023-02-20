// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.*;
import frc.robot.led.LED;
import frc.robot.led.commands.*;
import frc.robot.led.patterns.*;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.*;
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
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledStrip;

  private final ArrayList<CANTestable> testables = new ArrayList<CANTestable>();

  public RobotContainer() {
    PowerDistribution pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    SmartDashboard.putData(pdp);

    if (kIntakeEnabled) {
      configureIntake();
      testables.add(intakeSubsystem);
    }
    if (kSwerveEnabled) {
      configureSwerve();
      testables.add(swerveDrive);
    }
    if (kElevatorEnabled) {
      configureElevator();
      testables.add(elevatorSubsystem);
    }
    if (kArmEnabled) {
      configureArm();
      testables.add(armSubsystem);
    }
    if (kLedStripEnabled) {
      configureLEDStrip();
    }
  }

  private void configureIntake() {
    intakeSubsystem = new Intake();

    driver.leftBumper().whileTrue(new IntakeCube(intakeSubsystem));
    driver.leftTrigger().whileTrue(new IntakeCone(intakeSubsystem));
  }

  private void configureSwerve() {
    swerveDrive = new SwerveDrive();

    if (kElevatorEnabled) {
      // Enable elevator acceleration limiting
      swerveDrive.setDefaultCommand(
          new TeleopSwerve(
              swerveDrive,
              elevatorSubsystem,
              () -> driver.getLeftY(),
              () -> driver.getLeftX(),
              () -> driver.getRightX(),
              kFieldRelative,
              kOpenLoop));
    } else {
      swerveDrive.setDefaultCommand(
          new TeleopSwerve(
              swerveDrive,
              () -> driver.getLeftY(),
              () -> driver.getLeftX(),
              () -> driver.getRightX(),
              kFieldRelative,
              kOpenLoop));
    }

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

  public void configureElevator() {
    elevatorSubsystem = new Elevator();

    operator.a().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH));
    operator.b().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID));
    operator.x().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW));
  }

  private void configureArm() {
    armSubsystem = new Arm();
    // TODO: set button bindings for arm testing
  }

  public void configureLEDStrip() {
    ledStrip = new LED(0, new int[] {100});
    driver.a().onTrue(new LEDToggleGamePieceDisplay(ledStrip));
    driver.b().onTrue(new LEDSetAllSectionsPattern(ledStrip, new ColorChaseBluePattern()));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void test() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : testables) result &= subsystem.CANTest();
    System.out.println("CAN fully connected: " + result);
  }

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveDrive, armSubsystem);
    pitSubsystemRoutine.pitRoutine();
  }
}
