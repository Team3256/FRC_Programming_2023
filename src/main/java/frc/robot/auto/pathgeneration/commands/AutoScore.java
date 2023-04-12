// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;
import static frc.robot.led.LEDConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.arm.Arm;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.led.commands.SetAllColor;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoScore extends ParentCommand {
  public enum GridScoreHeight {
    HIGH,
    MID,
    LOW
  }

  private SwerveDrive swerveSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;
  private Intake intakeSubsystem;
  private GridScoreHeight gridScoreHeight;
  private BooleanSupplier cancelCommand;
  private BooleanSupplier isCurrentLEDPieceCone;
  private BooleanSupplier isAutoScoreMode;

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      GridScoreHeight gridScoreHeight,
      BooleanSupplier isCurrentLEDPieceCone,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier cancelCommand) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gridScoreHeight = gridScoreHeight;
    this.isAutoScoreMode = isAutoScoreMode;
    this.isCurrentLEDPieceCone = isCurrentLEDPieceCone;
    this.cancelCommand = cancelCommand;
  }

  @Override
  public void initialize() {
    System.out.println(
        "Is running auto score instead of presets: " + isAutoScoreMode.getAsBoolean());
    if (!isAutoScoreMode.getAsBoolean()) {
      switch (gridScoreHeight) {
        case HIGH:
          Commands.parallel(
                  new ConditionalCommand(
                      new SetEndEffectorState(
                          elevatorSubsystem,
                          armSubsystem,
                          SetEndEffectorState.EndEffectorPreset.SCORE_CONE_HIGH),
                      new SetEndEffectorState(
                          elevatorSubsystem,
                          armSubsystem,
                          SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_HIGH),
                      isCurrentLEDPieceCone))
              .schedule();
          break;
        default:
        case MID:
          Commands.parallel(
                  new ConditionalCommand(
                      new SetEndEffectorState(
                          elevatorSubsystem,
                          armSubsystem,
                          SetEndEffectorState.EndEffectorPreset.SCORE_CONE_MID),
                      new SetEndEffectorState(
                          elevatorSubsystem,
                          armSubsystem,
                          SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_MID),
                      isCurrentLEDPieceCone))
              .schedule();
          break;
        case LOW:
          new SetEndEffectorState(
                  elevatorSubsystem,
                  armSubsystem,
                  SetEndEffectorState.EndEffectorPreset.SCORE_ANY_LOW)
              .schedule();
          break;
      }

      return;
    }

    Pose2d start = swerveSubsystem.getPose();

    // Get scoring location id from SD
    int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
    if (DriverStation.getAlliance() == Alliance.Blue) {
      locationId = 8 - locationId;
    }
    if (0 > locationId || locationId > 8) {
      System.out.println("locationId was invalid (" + locationId + ")");
      new SetAllBlink(ledSubsystem, kError).withTimeout(6).schedule();
      return;
    }

    // Move to scoring waypoint
    Pose2d scoringWaypoint = kBlueScoreWaypointPoses[locationId];
    GamePiece scoringGamePiece = kScoringLocationPiece[locationId];

    System.out.println("Running: Go to grid (id: " + locationId + ") from " + start);
    if (DriverStation.getAlliance() == Alliance.Red) {
      scoringWaypoint = PathUtil.flip(scoringWaypoint);
    }
    Command moveToScoringWaypoint;
    if (kDynamicPathGenerationEnabled) {
      DynamicPathGenerator gen = new DynamicPathGenerator(start, scoringWaypoint, swerveSubsystem);
      moveToScoringWaypoint = gen.getCommand();
    } else
      moveToScoringWaypoint =
          PathGeneration.createDynamicAbsolutePath(
              start, scoringWaypoint, swerveSubsystem, kWaypointPathConstraints);

    BooleanSupplier isCurrentPieceCone = () -> scoringGamePiece.equals(GamePiece.CONE);
    Command runOuttake =
        new ConditionalCommand(
            new IntakeCube(intakeSubsystem, ledSubsystem),
            new IntakeCone(intakeSubsystem, ledSubsystem),
            isCurrentPieceCone);
    // Command stow = new StowArmElevator(elevatorSubsystem, armSubsystem);
    // Set arm and elevator command and end pose based on node type and height
    Pose2d scoringLocation;
    Command moveArmElevatorToPreset;

    switch (gridScoreHeight) {
      case HIGH:
        scoringLocation = kHighBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CONE_HIGH),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_HIGH),
                isCurrentPieceCone);
        break;
      case MID:
        scoringLocation = kMidBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CONE_MID),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_MID),
                isCurrentPieceCone);
        break;
      case LOW:
      default:
        scoringLocation = kBottomBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new SetEndEffectorState(
                elevatorSubsystem,
                armSubsystem,
                SetEndEffectorState.EndEffectorPreset.SCORE_ANY_LOW);
    }

    if (DriverStation.getAlliance() == Alliance.Red) {
      scoringLocation = PathUtil.flip(scoringLocation);
    }

    Command moveToScoringLocation =
        PathGeneration.createDynamicAbsolutePath(
            scoringWaypoint, scoringLocation, swerveSubsystem, kPathToDestinationConstraints);

    Command successLEDs = new SetAllBlink(ledSubsystem, kSuccess).withTimeout(5);

    Command errorLEDs = new SetAllBlink(ledSubsystem, kError).withTimeout(5);
    Command runningLEDs =
        new ConditionalCommand(
            new SetAllColor(ledSubsystem, kCone),
            new SetAllColor(ledSubsystem, kCube),
            isCurrentPieceCone);

    Command autoScore =
        Commands.sequence(
                moveToScoringWaypoint,
                Commands.parallel(moveToScoringLocation, moveArmElevatorToPreset))
            .deadlineWith(runningLEDs.asProxy())
            .finallyDo(
                (interrupted) -> {
                  if (!interrupted) successLEDs.schedule();
                })
            .until(cancelCommand)
            .handleInterrupt(errorLEDs::schedule);

    addChildCommands(autoScore);
  }
}
