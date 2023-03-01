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
    if (kIntakeEnabled) {
      configureIntake();
      canBusTestables.add(intakeSubsystem);
      loggables.add(intakeSubsystem);
    }
    if (kSwerveEnabled) {
      configureSwerve();
      canBusTestables.add(swerveSubsystem);
      loggables.add(swerveSubsystem);
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
    swerveSubsystem = new SwerveDrive();
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

    driver.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
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

    if (kElevatorEnabled && kArmEnabled) {
      Supplier<Command> scoreHighGrid =
          () ->
              DynamicPathFollower.run(swerveSubsystem, GoalType.HIGH_GRID, ledStrip)
                  .deadlineWith(new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy())
                  .andThen(getScoreCommand(GoalType.HIGH_GRID).asProxy());
      Supplier<Command> scoreMidGrid =
          () ->
              DynamicPathFollower.run(swerveSubsystem, GoalType.MID_GRID, ledStrip)
                  .deadlineWith(new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy())
                  .andThen(getScoreCommand(GoalType.MID_GRID).asProxy());
      Supplier<Command> scoreLowGrid =
          () ->
              DynamicPathFollower.run(swerveSubsystem, GoalType.LOW_GRID, ledStrip)
                  .deadlineWith(new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy())
                  .andThen(getScoreCommand(GoalType.LOW_GRID).asProxy());

      Supplier<Command> goToDoubleStationTop =
          () ->
              DynamicPathFollower.run(swerveSubsystem, GoalType.DOUBLE_STATION_TOP, ledStrip)
                  .deadlineWith(new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy());
      Supplier<Command> goToDoubleStationBottom =
          () ->
              DynamicPathFollower.run(swerveSubsystem, GoalType.DOUBLE_STATION_BOTTOM, ledStrip)
                  .deadlineWith(new StowArmElevator(elevatorSubsystem, armSubsystem).asProxy());

      driver.rightTrigger().onTrue(new InstantCommand(() -> scoreHighGrid.get().schedule()));
      driver.rightBumper().onTrue(new InstantCommand(() -> scoreMidGrid.get().schedule()));
      driver.b().onTrue(new InstantCommand(() -> scoreLowGrid.get().schedule()));
      driver.x().onTrue(new InstantCommand(() -> goToDoubleStationTop.get().schedule()));
      driver.y().onTrue(new InstantCommand(() -> goToDoubleStationBottom.get().schedule()));
    }
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
    intakeSubsystem = new Intake();

    operator.leftTrigger().whileTrue(new IntakeCube(intakeSubsystem));
    operator.rightTrigger().whileTrue(new IntakeCone(intakeSubsystem));
  }

  public void configureElevator() {
    elevatorSubsystem = new Elevator();

    if (kArmEnabled) {
      // TODO: comment out flaccid during match
      // elevatorSubsystem.setDefaultCommand(new ZeroElevator(elevatorSubsystem));

      // operator
      // .leftBumper()
      // .onTrue(
      // new ParallelCommandGroup(
      // new InstantCommand(armSubsystem::setArmFlaccid),
      // new InstantCommand(elevatorSubsystem::setElevatorFlaccid)));

      (operator.leftTrigger())
          .or(operator.rightTrigger())
          .onTrue(
              new ParallelCommandGroup(
                  new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
                  new SetArmAngle(armSubsystem, ArmPosition.GROUND_INTAKE)));

      driver
          .leftTrigger()
          .or(operator.a())
          .onTrue(new StowArmElevator(elevatorSubsystem, armSubsystem));

      // driver
      // .b()
      // .onTrue(
      // new ParallelCommandGroup(
      // new SetElevatorHeight(elevatorSubsystem,
      // Elevator.ElevatorPosition.ANY_PIECE_LOW),
      // new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW)));

      // driver
      // .rightBumper()
      // .onTrue(
      // new ParallelCommandGroup(
      // new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.ANY_PIECE_MID),
      // new ConditionalCommand(
      // new SetArmAngle(armSubsystem, ArmPosition.CONE_MID),
      // new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
      // this::isCurrentPieceCone)));

      // driver
      // .rightTrigger()
      // .onTrue(
      // new ParallelCommandGroup(
      // new ConditionalCommand(
      // new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CONE_HIGH),
      // new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CUBE_HIGH),
      // this::isCurrentPieceCone),
      // new ConditionalCommand(
      // new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
      // new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
      // this::isCurrentPieceCone)));

      // driver
      // .leftTrigger()
      // .onTrue(
      // new ParallelCommandGroup(
      // new SetArmAngle(armSubsystem, ArmPosition.DOUBLE_SUBSTATION),
      // new SetElevatorHeight(elevatorSubsystem,
      // ElevatorPosition.DOUBLE_SUBSTATION)));
    }
  }

  private void configureArm() {
    armSubsystem = new Arm();
    // armSubsystem.setDefaultCommand(new SetArmAngle(armSubsystem,
    // ArmPosition.DEFAULT));

    // TODO: comment out erect during match
    // operator.rightBumper().onTrue(new InstantCommand(armSubsystem::setArmErect));
  }

  public void configureLEDStrip() {
    ledStrip = new LED(0, new int[] {100});
    ledStrip.setDefaultCommand((new LEDSetAllSectionsPattern(ledStrip, new FIREPattern())));
    operator
        .leftBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new BlinkingConePattern()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCube));
    operator
        .rightBumper()
        .toggleOnTrue(new LEDSetAllSectionsPattern(ledStrip, new BlinkingCubePattern()))
        .toggleOnTrue(new InstantCommand(this::setPieceToCube));
  }

  public Command getAutonomousCommand() {
    Command setArmElevatorOnRightSide;
    if (kElevatorEnabled && kArmEnabled) {
      setArmElevatorOnRightSide =
          new ParallelRaceGroup(
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

    for (Loggable device : loggables) device.logInit();
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
    for (CANTestable subsystem : canBusTestables) result &= subsystem.CANTest();
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
