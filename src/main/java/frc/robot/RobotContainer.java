// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeatureFlags.*;
import static frc.robot.led.LEDConstants.*;
import static frc.robot.swerve.SwerveConstants.kFieldRelative;
import static frc.robot.swerve.SwerveConstants.kOpenLoop;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.KeepArmAtPosition;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.commands.SetArmElevatorStart;
import frc.robot.auto.pathgeneration.commands.*;
import frc.robot.auto.pathgeneration.commands.AutoIntakeAtDoubleSubstation.SubstationLocation;
import frc.robot.climb.Climb;
import frc.robot.climb.commands.DeployClimb;
import frc.robot.climb.commands.RetractClimb;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.LatchGamePiece;
import frc.robot.led.LED;
import frc.robot.led.commands.*;
import frc.robot.led.patterns.*;
import frc.robot.led.patterns.Blink.ConePatternBlink;
import frc.robot.led.patterns.Blink.CubePatternBlink;
import frc.robot.led.patterns.Blink.LimitedSwerveBlink;
import frc.robot.logging.Loggable;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerveX;
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
  public enum GamePiece {
    CUBE,
    CONE
  }

  public enum Mode {
    AUTO_SCORE,
    PRESET
  }

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Climb climbSubsystem;
  private LED ledStrip;
  private GamePiece currentPiece = GamePiece.CUBE;
  private SubstationLocation doubleSubstationLocation = SubstationLocation.RIGHT_SIDE;
  private SendableChooser<Mode> modeChooser;

  private AutoPaths autoPaths;

  private final ArrayList<CANTestable> canBusTestables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    if (kArmEnabled) armSubsystem = new Arm();
    if (kIntakeEnabled) intakeSubsystem = new Intake();
    if (kElevatorEnabled) elevatorSubsystem = new Elevator();
    if (kSwerveEnabled) swerveSubsystem = new SwerveDrive();
    if (kClimbEnabled) climbSubsystem = new Climb();
    if (kLedStripEnabled) ledStrip = new LED(kPort, new int[] {100});

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
    if (kClimbEnabled) {
      configureClimb();
      canBusTestables.add(climbSubsystem);
    }
    if (kLedStripEnabled) {
      configureLEDStrip();
      loggables.add(ledStrip);
    }

    modeChooser = new SendableChooser<>();
    if (FeatureFlags.kAutoScoreEnabled) {
      modeChooser.setDefaultOption("Auto Score", Mode.AUTO_SCORE);
      modeChooser.addOption("Presets", Mode.PRESET);
    } else {
      modeChooser.setDefaultOption("Presets", Mode.PRESET);
      modeChooser.addOption("Auto Score", Mode.AUTO_SCORE);
    }
    SmartDashboard.putData("Auto Score Toggle", modeChooser);

    autoPaths = new AutoPaths(swerveSubsystem, intakeSubsystem, elevatorSubsystem, armSubsystem);
    autoPaths.sendCommandsToChooser();

    if (AutoConstants.kAutoDebug) {
      PathPlannerServer.startServer(5811);
    }

    SmartDashboard.putString(
        "Current Double Substation Location", doubleSubstationLocation.toString());
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
                () -> isMovingJoystick(driver),
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
                () -> isMovingJoystick(driver),
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
                () -> isMovingJoystick(driver),
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
                () -> isMovingJoystick(driver),
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
                    kOpenLoop)
                .deadlineWith(
                    new LEDSetAllSectionsPattern(
                        // TODO fix later
                        ledStrip, new LimitedSwerveBlink(this::isCurrentPieceCone))));

    driver
        .x()
        .onTrue(
            new LockSwerveX(swerveSubsystem)
                .andThen(new LEDSetAllSectionsPattern(ledStrip, new LockSwervePattern())));

    if (kElevatorEnabled && kArmEnabled && kLedStripEnabled) {
      driver
          .leftTrigger()
          .onTrue(
              new AutoIntakeAtDoubleSubstation(
                  swerveSubsystem,
                  intakeSubsystem,
                  elevatorSubsystem,
                  armSubsystem,
                  ledStrip,
                  () -> doubleSubstationLocation,
                  () -> isMovingJoystick(driver),
                  () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
                  this::isCurrentPieceCone));

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
                  this::isCurrentPieceCone,
                  () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
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
                  this::isCurrentPieceCone,
                  () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
                  () -> isMovingJoystick(driver)));
    }

    operator.a().toggleOnTrue(new InstantCommand(this::toggleSubstationLocation));
  }

  private void configureIntake() {
    new Trigger(intakeSubsystem::isCurrentSpiking)
        .onTrue(new LatchGamePiece(intakeSubsystem, this::isCurrentPieceCone));

    (operator.rightTrigger())
        .or(driver.b())
        .whileTrue(
            new ConditionalCommand(
                new IntakeCube(intakeSubsystem),
                new IntakeCone(intakeSubsystem),
                this::isCurrentPieceCone));
  }

  public void configureElevator() {
    if (kArmEnabled) {
      driver
          .y()
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

  public void configureClimb() {
    operator.start().whileTrue(new DeployClimb(climbSubsystem));
    operator.back().whileTrue(new RetractClimb(climbSubsystem));
  }

  public void configureLEDStrip() {
    ledStrip.setDefaultCommand((new LEDSetAllSectionsPattern(ledStrip, new FIREPattern())));

    operator
        .rightBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new ConePatternBlink()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCone));
    operator
        .leftBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new CubePatternBlink()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCube));
  }

  public Command getAutonomousCommand() {
    Command autoPath = autoPaths.getSelectedPath();
    Command setArmElevatorStart;
    if (kElevatorEnabled && kArmEnabled) {
      setArmElevatorStart = new SetArmElevatorStart(elevatorSubsystem, armSubsystem);

      return Commands.sequence(
          setArmElevatorStart.asProxy(),
          autoPath,
          Commands.parallel(
              new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy(),
              new LockSwerveX(swerveSubsystem)));
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

  public boolean isMovingJoystick(CommandXboxController controller) {
    return Math.abs(controller.getLeftX()) > kStickCancelDeadband
        || Math.abs(controller.getLeftY()) > kStickCancelDeadband
        || Math.abs(controller.getRightX()) > kStickCancelDeadband
        || Math.abs(controller.getRightY()) > kStickCancelDeadband;
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return null;
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : canBusTestables) result &= subsystem.CANTest();
    System.out.println("CAN fully connected: " + result);
    return result;
  }

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveSubsystem, armSubsystem);
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

  public void toggleSubstationLocation() {
    if (doubleSubstationLocation == SubstationLocation.LEFT_SIDE) {
      doubleSubstationLocation = SubstationLocation.RIGHT_SIDE;
    } else {
      doubleSubstationLocation = SubstationLocation.LEFT_SIDE;
    }

    SmartDashboard.putString(
        "Current Double Substation Location", doubleSubstationLocation.toString());
  }

  public SubstationLocation getSubstationLocation() {
    return this.doubleSubstationLocation;
  }
}
