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

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.*;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.commands.SetArmElevatorStart;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower.GoalType;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPosition;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.*;
import frc.robot.led.patterns.*;
import frc.robot.logging.DoubleSendable;
import frc.robot.logging.Loggable;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.*;
import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements CANTestable, Loggable {
  public enum Piece {
    CUBE,
    CONE
  }

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledStrip;
  private Piece currentPiece = Piece.CUBE;

  private AutoPaths autoPaths;

  private final ArrayList<CANTestable> canBusTestables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    if (kIntakeEnabled)
      intakeSubsystem = new Intake();
    if (kArmEnabled)
      armSubsystem = new Arm();
    if (kElevatorEnabled)
      elevatorSubsystem = new Elevator();
    if (kSwerveEnabled)
      swerveSubsystem = new SwerveDrive();
    if (kLedStripEnabled)
      ledStrip = new LED(0, new int[] { 100 });

    if (kIntakeEnabled) {
      configureIntake();
      canBusTestables.add(intakeSubsystem);
      loggables.add(intakeSubsystem);
    }
    if (kArmEnabled) {
      configureArm();
      canBusTestables.add(armSubsystem);
      loggables.add(armSubsystem);
    }
    if (kElevatorEnabled) {
      configureElevator();
      canBusTestables.add(elevatorSubsystem);
      loggables.add(elevatorSubsystem);
    }
    if (kSwerveEnabled) {
      configureSwerve();
      canBusTestables.add(swerveSubsystem);
      loggables.add(swerveSubsystem);
    }
    if (kLedStripEnabled) {
      configureLEDStrip();
      loggables.add(ledStrip);
    }

    autoPaths = new AutoPaths(swerveSubsystem, intakeSubsystem, elevatorSubsystem, armSubsystem);
    autoPaths.sendCommandsToChooser();

    if (AutoConstants.kAutoDebug) {
      PathPlannerServer.startServer(5811);
    }
  }

  private void configureSwerve() {
    // Good
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            kFieldRelative,
            kOpenLoop));

    // good
    driver
        .povUp()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 0,
                () -> -1,
                () -> isRotating(driver),
                kFieldRelative,
                kOpenLoop));

    // good
    driver
        .povDown()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 0,
                () -> 1,
                () -> isRotating(driver),
                kFieldRelative,
                kOpenLoop));
    // good
    driver
        .povRight()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 1,
                () -> 0,
                () -> isRotating(driver),
                kFieldRelative,
                kOpenLoop));

    // good
    driver
        .povLeft()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> -1,
                () -> 0,
                () -> isRotating(driver),
                kFieldRelative,
                kOpenLoop));

    // good
    driver.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    // good
    driver
        .leftBumper()
        .toggleOnTrue(
            new TeleopSwerveLimited(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                driver::getRightX,
                kFieldRelative,
                kOpenLoop));
  }

  private Command getScoreCommand(GoalType goalType) {
    switch (goalType) {
      case HIGH_GRID:
        return new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CUBE_HIGH),
                this::isCurrentPieceCone),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
                new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
                this::isCurrentPieceCone),
            new WaitCommand(2)
                .andThen(
                    new ConditionalCommand(
                        new IntakeCube(intakeSubsystem),
                        new IntakeCube(intakeSubsystem),
                        this::isCurrentPieceCone)));
      case MID_GRID:
        return new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.ANY_PIECE_MID),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPosition.CONE_MID),
                new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
                this::isCurrentPieceCone),
            new WaitCommand(2)
                .andThen(
                    new ConditionalCommand(
                        new IntakeCube(intakeSubsystem),
                        new IntakeCube(intakeSubsystem),
                        this::isCurrentPieceCone)));
      case LOW_GRID:
        return new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.ANY_PIECE_LOW),
            new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW),
            new WaitCommand(2)
                .andThen(
                    new ConditionalCommand(
                        new IntakeCube(intakeSubsystem),
                        new IntakeCube(intakeSubsystem),
                        this::isCurrentPieceCone)));
      default:
        return new InstantCommand();
    }
  }

  private void configureIntake() {
    operator
        .leftTrigger()
        .whileTrue(new IntakeCube(intakeSubsystem))
        .whileTrue(new InstantCommand(this::setPieceToCube));

    operator
        .rightTrigger()
        .whileTrue(new IntakeCone(intakeSubsystem))
        .whileTrue(new InstantCommand(this::setPieceToCone));
  }

  public void configureElevator() {

    if (kArmEnabled) {
      (operator.leftTrigger())
          .or(operator.rightTrigger())
          .onTrue(
              new ParallelCommandGroup(
                  new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
                  new SetArmAngle(armSubsystem, ArmPosition.GROUND_INTAKE)));

      driver.leftTrigger().onTrue(new StowArmElevator(elevatorSubsystem, armSubsystem));
    }
  }

  private void configureArm() {
  }

  public void configureLEDStrip() {
    ledStrip.setDefaultCommand((new LEDSetAllSectionsPattern(ledStrip, new FIREPattern())));
    operator
        .rightBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new BlinkingConePattern()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCone));
    operator
        .leftBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new BlinkingCubePattern()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCube));
  }

  public Command getAutonomousCommand() {
    Command setArmElevatorOnRightSide;
    if (kElevatorEnabled && kArmEnabled) {
      setArmElevatorOnRightSide = new ParallelRaceGroup(
          new WaitCommand(1.5), new SetArmElevatorStart(elevatorSubsystem, armSubsystem));
    } else {
      setArmElevatorOnRightSide = new InstantCommand();
    }
    Command autoPath = autoPaths.getSelectedPath();
    return setArmElevatorOnRightSide.asProxy().andThen(autoPath);
  }

  @Override
  public void logInit() {
    SmartDashboard.putData("trajectoryViewer", trajectoryViewer);
    SmartDashboard.putData("waypointViewer", waypointViewer);
    SmartDashboard.putData("swerveViewer", swerveViewer);

    for (Loggable device : loggables)
      device.logInit();
    Shuffleboard.getTab(kDriverTabName)
        .add(
            "Joystick",
            new DoubleSendable(
                () -> Math.toDegrees(Math.atan2(driver.getRightX(), driver.getRightY())), "Gyro"));
  }

  public boolean isRotating(CommandXboxController controller) {
    return Math.abs(controller.getRightX()) > 0.3; // threshold
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return null;
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : canBusTestables)
      result &= subsystem.CANTest();
    System.out.println("CAN fully connected: " + result);
    return result;
  }

  public void startPitRoutine() {
    // PitTestRoutine pitSubsystemRoutine =
    // new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveSubsystem,
    // armSubsystem);
    // pitSubsystemRoutine.pitRoutine();
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
