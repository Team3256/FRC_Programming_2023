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
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.KeepArmAtPosition;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.commands.SetArmElevatorStart;
import frc.robot.auto.dynamicpathgeneration.DynamicPathFollower.GoalType;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPreset;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.LatchGamePiece;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.BlinkingConePattern;
import frc.robot.led.patterns.BlinkingCubePattern;
import frc.robot.led.patterns.FIREPattern;
import frc.robot.limelight.AlignWithGamePiece;
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
    if (kArmEnabled) armSubsystem = new Arm();
    if (kIntakeEnabled) intakeSubsystem = new Intake();
    if (kElevatorEnabled) elevatorSubsystem = new Elevator();
    if (kSwerveEnabled) swerveSubsystem = new SwerveDrive();
    if (kLedStripEnabled) ledStrip = new LED(0, new int[] {100});

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

    //    driver.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyroYaw));
    System.out.println("**********************");
    System.out.println("ConfigureSwerve");
    System.out.println("**********************");
    driver.a().onTrue(new AlignWithGamePiece(swerveSubsystem, 0));

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
    // driver.x().whileTrue(new AutoBalance(swerveSubsystem));
  }

  private Command getScoreCommand(GoalType goalType) {
    switch (goalType) {
      case HIGH_GRID:
        return new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPreset.CONE_HIGH),
                new SetElevatorHeight(elevatorSubsystem, ElevatorPreset.CUBE_HIGH),
                this::isCurrentPieceCone),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPreset.CONE_HIGH),
                new SetArmAngle(armSubsystem, ArmPreset.CUBE_HIGH),
                this::isCurrentPieceCone));
      case MID_GRID:
        return new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, ElevatorPreset.ANY_PIECE_MID),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, ArmPreset.CONE_MID),
                new SetArmAngle(armSubsystem, ArmPreset.CUBE_MID),
                this::isCurrentPieceCone));
      case LOW_GRID:
        return new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, ElevatorPreset.ANY_PIECE_LOW),
            new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW));
      default:
        return new InstantCommand();
    }
  }

  private void configureIntake() {
    intakeSubsystem.setDefaultCommand(
        new LatchGamePiece(intakeSubsystem, this::isCurrentPieceCone));
    (operator.rightTrigger())
        .whileTrue(
            new ConditionalCommand(
                new IntakeCone(intakeSubsystem),
                new IntakeCube(intakeSubsystem),
                this::isCurrentPieceCone));
  }

  public void configureElevator() {
    if (kArmEnabled && kIntakeEnabled) {
      driver.rightTrigger().onTrue(getScoreCommand(GoalType.HIGH_GRID));
      driver.rightBumper().onTrue(getScoreCommand(GoalType.MID_GRID));
      driver.b().onTrue(getScoreCommand(GoalType.LOW_GRID));

      driver
          .leftTrigger()
          .or(operator.leftTrigger())
          .onTrue(new StowArmElevator(elevatorSubsystem, armSubsystem));

      operator
          .b()
          .or(driver.y())
          .toggleOnTrue(
              new ParallelCommandGroup(
                  // TODO need 5.5 deg for cone, lower (4.5?) for cube
                  new SetElevatorHeight(elevatorSubsystem, ElevatorPreset.DOUBLE_SUBSTATION),
                  new SetArmAngle(armSubsystem, ArmPreset.DOUBLE_SUBSTATION),
                  new ConditionalCommand(
                      new IntakeCone(intakeSubsystem, ledStrip),
                      new IntakeCube(intakeSubsystem, ledStrip),
                      this::isCurrentPieceCone)));
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
      setArmElevatorOnRightSide =
          new ParallelRaceGroup(
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
    return Piece.CONE.equals(currentPiece);
  }

  public void setPieceToCone() {
    currentPiece = Piece.CONE;
  }

  public void setPieceToCube() {
    currentPiece = Piece.CUBE;
  }
}
