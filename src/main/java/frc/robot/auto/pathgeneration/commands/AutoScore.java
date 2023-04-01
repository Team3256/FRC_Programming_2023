// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPreset;
import frc.robot.elevator.commands.SetElevatorExtension;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.*;
import frc.robot.led.patterns.Blink.ErrorPatternBlink;
import frc.robot.led.patterns.Blink.SuccessPatternBlink;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoScore extends CommandBase {
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

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      GridScoreHeight gridScoreHeight,
      BooleanSupplier cancelCommand) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gridScoreHeight = gridScoreHeight;
    this.cancelCommand = cancelCommand;
  }

  @Override
  public void initialize() {
    Pose2d start = swerveSubsystem.getPose();

    // Uncomment if testing random start pose:
    // Pose2d start = new Pose2d(
    // Math.random() * 0.5 + 1.9,
    // Math.random() * 4.4 + 0.54,
    // Rotation2d.fromDegrees(Math.random() * 180));

    // Get scoring location id from SD
    int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
    if (DriverStation.getAlliance() == Alliance.Blue) {
      locationId = 8 - locationId;
    }
    if (0 > locationId || locationId > 8) {
      System.out.println("locationId was invalid (" + locationId + ")");
      new LEDSetAllSectionsPattern(ledSubsystem, new ErrorPatternBlink()).withTimeout(6).schedule();
      return;
    }

    // Move to scoring waypoint
    Pose2d scoringWaypoint = kBlueScoreWaypointPoses[locationId];
    GamePiece scoringGamePiece = kScoringLocationPiece[locationId];
    BooleanSupplier isCurrentPieceCone = () -> scoringGamePiece.equals(GamePiece.CONE);

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

    Command runOuttake =
        new ConditionalCommand(
            new IntakeCube(intakeSubsystem, ledSubsystem),
            new IntakeCone(intakeSubsystem, ledSubsystem),
            isCurrentPieceCone);
    Command stow = new StowArmElevator(elevatorSubsystem, armSubsystem);
    // Set arm and elevator command and end pose based on node type and height
    Pose2d scoringLocation;
    Command moveArmElevatorToPreset;

    switch (gridScoreHeight) {
      case HIGH:
        scoringLocation = kHighBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new SetElevatorExtension(elevatorSubsystem, ElevatorPreset.CONE_HIGH),
                    new SetElevatorExtension(elevatorSubsystem, ElevatorPreset.CUBE_HIGH),
                    isCurrentPieceCone),
                new ConditionalCommand(
                    new SetArmAngle(armSubsystem, ArmPreset.CONE_HIGH),
                    new SetArmAngle(armSubsystem, ArmPreset.CUBE_HIGH),
                    isCurrentPieceCone));
        break;
      case MID:
        scoringLocation = kMidBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new SetElevatorExtension(elevatorSubsystem, ElevatorPreset.ANY_PIECE_MID),
                new ConditionalCommand(
                    new SetArmAngle(armSubsystem, ArmPreset.CONE_MID),
                    new SetArmAngle(armSubsystem, ArmPreset.CUBE_MID),
                    isCurrentPieceCone));
        break;
      case LOW:
      default:
        scoringLocation = kBottomBlueScoringPoses[locationId];
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new SetElevatorExtension(elevatorSubsystem, ElevatorPreset.ANY_PIECE_LOW),
                new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW));
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      scoringLocation = PathUtil.flip(scoringLocation);
    }

    // Move to scoring location
    Command moveToScoringLocation =
        PathGeneration.createDynamicAbsolutePath(
            scoringWaypoint, scoringLocation, swerveSubsystem, kPathToDestinationConstraints);

    // LED verbose
    Command successLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPatternBlink()).withTimeout(5);
    Command errorLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new ErrorPatternBlink()).withTimeout(5);
    Command runningLEDs =
        new ConditionalCommand(
            new LEDSetAllSectionsPattern(ledSubsystem, new ConePattern()),
            new LEDSetAllSectionsPattern(ledSubsystem, new CubePattern()),
            isCurrentPieceCone);

    // schedule final composed command
    Command autoScore =
        Commands.sequence(
                moveToScoringWaypoint,
                Commands.parallel(moveToScoringLocation, moveArmElevatorToPreset))
            .deadlineWith(runningLEDs.asProxy())
            .finallyDo((interrupted) -> successLEDs.schedule())
            .until(cancelCommand)
            .handleInterrupt(errorLEDs::schedule);

    autoScore.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
