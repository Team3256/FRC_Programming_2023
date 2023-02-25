// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.swerve.SwerveConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.*;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPosition;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.*;
import frc.robot.led.LED;
import frc.robot.led.commands.*;
import frc.robot.led.patterns.*;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.*;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements CANTestable, Loggable {
  public enum Piece {
    CUBE,
    CONE
  }

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDrive swerveDrive;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledStrip;
  private Piece currentPiece = Piece.CONE;

  private final ArrayList<CANTestable> testables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
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
    if (kLedStripEnabled) {
      configureLEDStrip();
      loggables.add(ledStrip);
    }
  }

  private void configureSwerve() {
    swerveDrive = new SwerveDrive();

    driver.leftTrigger().onTrue(new InstantCommand(swerveDrive::zeroGyro));
    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> driver.getRightX(),
            kFieldRelative,
            kOpenLoop));

    // driver
    // .rightBumper()
    // .whileTrue(
    // new TeleopSwerveWithAzimuth(
    // swerveDrive,
    // () -> driver.getRightY(),
    // () -> driver.getRightX(),
    // () -> driver.getLeftX(),
    // () -> driver.getLeftY(),
    // kFieldRelative,
    // kOpenLoop));

    // driver
    // .b()
    // .toggleOnTrue(
    // new TeleopSwerveLimited(
    // swerveDrive,
    // () -> driver.getRightY(),
    // () -> driver.getRightX(),
    // () -> driver.getLeftX(),
    // kFieldRelative,
    // kOpenLoop));
  }

  private void configureIntake() {
    intakeSubsystem = new Intake();

    driver.rightBumper().whileTrue(new IntakeCube(intakeSubsystem));
    driver.rightBumper().onTrue(new InstantCommand(this::setPieceToCube));
    driver.leftBumper().whileTrue(new IntakeCone(intakeSubsystem));
    driver.leftBumper().onTrue(new InstantCommand(this::setPieceToCone));
  }

  public void configureElevator() {
    elevatorSubsystem = new Elevator();

    driver.x().whileTrue(new ZeroElevator(elevatorSubsystem));

    driver
        .y()
        .onTrue(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CUBE_HIGH),
                this::isCurrentPieceCone));
    driver
        .b()
        .onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_MID));
    driver
        .a()
        .onTrue(new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.ANY_PIECE_LOW));

    driver.x().onTrue(new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.GROUND_INTAKE));
  }

  private void configureArm() {
    armSubsystem = new Arm();
    driver
        .y()
        .onTrue(
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
                new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
                this::isCurrentPieceCone));
    driver
        .b()
        .onTrue(
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPosition.CONE_MID),
                new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
                this::isCurrentPieceCone));
    driver.a().onTrue(new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW));
    driver.x().onTrue(new SetArmAngle(armSubsystem, ArmPosition.GROUND_INTAKE));

    operator.a().onTrue(new InstantCommand(armSubsystem::setArmLimp));
    operator.b().onTrue(new InstantCommand(armSubsystem::setArmErect));
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
    for (Loggable device : loggables) device.logInit();
    Shuffleboard.getTab(kDriverTabName)
        .add(
            "Joystick",
            new DoubleSendable(
                () -> Math.toDegrees(Math.atan2(driver.getRightX(), driver.getRightY())), "Gyro"));
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

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveDrive, armSubsystem);
    pitSubsystemRoutine.pitRoutine();
  }

  public boolean isCurrentPieceCone() {
    return Piece.CONE.equals(currentPiece);
  }

  public void setPieceToCone() {
    currentPiece = Piece.CONE;
  }

  public void setPieceToCube() {
    currentPiece = Piece.CUBE;
  }
}
