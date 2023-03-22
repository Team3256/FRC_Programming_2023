// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.swerve.SwerveConstants.kFieldRelative;
import static frc.robot.swerve.SwerveConstants.kOpenLoop;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.KeepArmAtPosition;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.commands.SetArmElevatorStart;
import frc.robot.auto.pathgeneration.commands.*;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.BlinkingConePattern;
import frc.robot.led.patterns.BlinkingCubePattern;
import frc.robot.led.patterns.FIREPattern;
import frc.robot.logging.Loggable;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerveX;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveLimited;
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;
import java.util.ArrayList;

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
  public enum GamePiece {
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
  private GamePiece currentPiece = GamePiece.CUBE;

  private AutoPaths autoPaths;

  private final ArrayList<CANTestable> canBusTestables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    if (kArmEnabled)
      armSubsystem = new Arm();
    if (kIntakeEnabled)
      intakeSubsystem = new Intake();
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
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            kFieldRelative,
            kOpenLoop));

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

    driver.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyroYaw));

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

    operator.x().onTrue(new LockSwerveX(swerveSubsystem));

    operator
        .b()
        .onTrue(
            new AutoIntakeAtSubstation(
                swerveSubsystem,
                intakeSubsystem,
                elevatorSubsystem,
                armSubsystem,
                ledStrip,
                AutoIntakeAtSubstation.SubstationLocation.LEFT_SIDE,
                () -> isMovingJoystick(driver),
                this::isCurrentPieceCone));

    // operator
    // .b()
    // .onTrue(
    // new AutoIntakeAtSubstation(
    // swerveSubsystem,
    // intakeSubsystem,
    // elevatorSubsystem,
    // armSubsystem,
    // ledStrip,
    // AutoIntakeAtSubstation.SubstationLocation.RIGHT_SIDE,
    // () -> isMovingJoystick(driver),
    // this::isCurrentPieceCone));
  }

  private void configureIntake() {
    // intakeSubsystem.setDefaultCommand(
    // new LatchGamePiece(intakeSubsystem, this::isCurrentPieceCone));
    (operator.rightTrigger())
        .whileTrue(
            new ConditionalCommand(
                new IntakeCube(intakeSubsystem),
                new IntakeCone(intakeSubsystem),
                this::isCurrentPieceCone));
  }

  public void configureElevator() {
    if (kArmEnabled && kIntakeEnabled) {
      if (kLedStripEnabled) {
        driver
            .rightTrigger()
            .onTrue(
                new AutoScore(
                    swerveSubsystem,
                    intakeSubsystem,
                    elevatorSubsystem,
                    armSubsystem,
                    ledStrip,
                    AutoScore.GridScoreHeight.HIGH,
                    () -> isMovingJoystick(driver)));
        driver
            .rightBumper()
            .onTrue(
                new AutoScore(
                    swerveSubsystem,
                    intakeSubsystem,
                    elevatorSubsystem,
                    armSubsystem,
                    ledStrip,
                    AutoScore.GridScoreHeight.MID,
                    () -> isMovingJoystick(driver)));
      }

      driver
          .leftTrigger()
          .or(operator.leftTrigger())
          .onTrue(new StowArmElevator(elevatorSubsystem, armSubsystem));
    }
  }

  private void configureArm() {
    armSubsystem.setDefaultCommand(new KeepArmAtPosition(armSubsystem));
    if (kIntakeEnabled && FeatureFlags.kOperatorManualArmControlEnabled) {
      operator.povUp().whileTrue(new SetArmVoltage(armSubsystem, ArmConstants.kManualArmVoltage));
      operator
          .povDown()
          .whileTrue(new SetArmVoltage(armSubsystem, -ArmConstants.kManualArmVoltage));
      operator
          .povUp()
          .or(operator.povDown())
          .whileTrue(
              new ConditionalCommand(
                  new IntakeCone(intakeSubsystem),
                  new IntakeCube(intakeSubsystem),
                  this::isCurrentPieceCone));
    }
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
    Command autoPath = autoPaths.getSelectedPath();
    Command setArmElevatorOnRightSide;
    if (kElevatorEnabled && kArmEnabled) {
      setArmElevatorOnRightSide = new ParallelRaceGroup(
          new WaitCommand(1.5), new SetArmElevatorStart(elevatorSubsystem, armSubsystem));

      return Commands.sequence(
          setArmElevatorOnRightSide.asProxy(),
          autoPath,
          new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy());
    } else {
      return autoPath;
    }
  }

  @Override
  public void logInit() {
    SmartDashboard.putData("trajectoryViewer", trajectoryViewer);
    SmartDashboard.putData("waypointViewer", waypointViewer);
    SmartDashboard.putData("swerveViewer", swerveViewer);
  }

  public boolean isRotating(CommandXboxController controller) {
    return Math.abs(controller.getRightX()) > kStickRotationThreshold; // threshold
  }

  public boolean isMovingJoystick(CommandXboxController controller) {
    return Math.abs(controller.getLeftX()) > kStickDeadband || Math.abs(controller.getLeftY()) > kStickDeadband
        || Math.abs(controller.getRightX()) > kStickDeadband || Math.abs(controller.getRightY()) > kStickDeadband;
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
    PitTestRoutine pitSubsystemRoutine = new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveSubsystem,
        armSubsystem);
    pitSubsystemRoutine.runPitRoutine();
  }

  public boolean isCurrentPieceCone() {
    return GamePiece.CONE.equals(currentPiece);
  }

  public void setPieceToCone() {
    currentPiece = GamePiece.CONE;
  }

  public void setPieceToCube() {
    currentPiece = GamePiece.CUBE;
  }
}
