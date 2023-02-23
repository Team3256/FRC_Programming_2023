// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.swerve.SwerveConstants.kFieldRelative;
import static frc.robot.swerve.SwerveConstants.kOpenLoop;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.DefaultArmElevatorDriveConfig;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower.GoalType;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.*;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.commands.LEDToggleGamePieceDisplay;
import frc.robot.led.patterns.ColorChaseBluePattern;
import frc.robot.logging.GyroSendable;
import frc.robot.logging.Loggable;
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
public class RobotContainer implements CANTestable, Loggable {

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDrive swerveDrive;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledStrip;

  private final ArrayList<CANTestable> testables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    SmartDashboard.putData("trajectoryViewer", trajectoryViewer);
    SmartDashboard.putData("waypointViewer", waypointViewer);

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
      testables.add(elevatorSubsystem);
      loggables.add(elevatorSubsystem);
    }
    if (kArmEnabled) {
      configureArm();
      testables.add(armSubsystem);
      loggables.add(armSubsystem);
    }
    if (kElevatorEnabled) {
      configureElevator();
      testables.add(elevatorSubsystem);
    }
    if (kLedStripEnabled) {
      configureLEDStrip();
      loggables.add(ledStrip);
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

    driver
        .y()
        .onTrue(new InstantCommand(() -> DynamicPathFollower.run(swerveDrive, GoalType.HIGH_GRID)));
    driver
        .x()
        .onTrue(new InstantCommand(() -> DynamicPathFollower.run(swerveDrive, GoalType.MID_GRID)));
    driver
        .b()
        .onTrue(new InstantCommand(() -> DynamicPathFollower.run(swerveDrive, GoalType.LOW_GRID)));
    driver
        .a()
        .onTrue(new InstantCommand(() -> DynamicPathFollower.run(swerveDrive, GoalType.STATION)));
  }

  public void configureElevator() {
    elevatorSubsystem = new Elevator();

    operator.a().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.HIGH));
    operator.b().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.MID));
    operator.x().onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.LOW));

    if (kArmEnabled) {
      operator.y().onTrue(new DefaultArmElevatorDriveConfig(elevatorSubsystem, armSubsystem));
    }
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

  @Override
  public void logInit() {
    PowerDistribution pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    Shuffleboard.getTab(kElectricalTabName).add(pdp);
    for (Loggable device : loggables) device.logInit();
    Shuffleboard.getTab(kDriverTabName)
        .add(
            "Joystick",
            new GyroSendable(
                () -> Math.toDegrees(Math.atan2(driver.getRightX(), driver.getRightY()))));
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return null;
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : testables) result &= subsystem.CANTest();
    System.out.println("CAN fully connected: " + result);
    return result;
  }

  public void periodic() {
    trajectoryViewer.setRobotPose(swerveDrive.getPose());
  }

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveDrive, armSubsystem);
    pitSubsystemRoutine.pitRoutine();
  }
}
