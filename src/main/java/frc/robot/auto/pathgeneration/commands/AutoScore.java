// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.Constants.FeatureFlags.kAutoOuttakeEnabled;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;
import static frc.robot.led.LEDConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.FeatureFlags;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.arm.Arm;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.OuttakeCone;
import frc.robot.intake.commands.OuttakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.led.commands.SetAllColor;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
  private Supplier<GridScoreHeight> gridScoreHeight;
  private BooleanSupplier cancelCommand;
  private BooleanSupplier isOperatorSelectingCone;
  private BooleanSupplier isAutoScoreMode;
  private boolean isScoringFront;

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      GridScoreHeight gridScoreHeight,
      BooleanSupplier isOperatorSelectingCone,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier cancelCommand,
      boolean isScoringFront) {

    this(
        swerveDrive,
        intakeSubsystem,
        elevatorSubsystem,
        armSubsystem,
        ledSubsystem,
        () -> gridScoreHeight,
        isOperatorSelectingCone,
        isAutoScoreMode,
        cancelCommand,
        isScoringFront);
  }

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      Supplier<GridScoreHeight> gridScoreHeight,
      BooleanSupplier isOperatorSelectingCone,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier cancelCommand,
      boolean isScoringFront) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gridScoreHeight = gridScoreHeight;
    this.isAutoScoreMode = isAutoScoreMode;
    this.isOperatorSelectingCone = isOperatorSelectingCone;
    this.cancelCommand = cancelCommand;
    this.isScoringFront = isScoringFront;

    addRequirements(swerveDrive, intakeSubsystem, elevatorSubsystem, armSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println(
        "Is running auto score instead of presets: " + isAutoScoreMode.getAsBoolean());
    Command moveArmElevatorToPreset;
    boolean isRedAlliance = DriverStation.getAlliance() == Alliance.Red;

    switch (gridScoreHeight.get()) {
      case HIGH:
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
                isOperatorSelectingCone);
        break;
      case MID:
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
                isOperatorSelectingCone);
        break;
      default:
        moveArmElevatorToPreset =
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_ANY_LOW_FRONT),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_ANY_LOW_BACK),
                () -> isScoringFront);
        break;
    }
    if (!isAutoScoreMode.getAsBoolean()) {
      addChildCommands(moveArmElevatorToPreset);
    } else {

      Pose2d start = swerveSubsystem.getPose();

      // Get scoring location id from SD
      int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
      if (DriverStation.getAlliance() == Alliance.Blue) {
        locationId = 8 - locationId;
      }
      if (0 > locationId || locationId > 8) {
        System.out.println("locationId was invalid (" + locationId + ")");
        new SetAllColor(ledSubsystem, kError).withTimeout(2.5).schedule();
        return;
      }

      // Move to scoring waypoint
      Pose2d scoringWaypoint = kBlueScoreWaypointPoses[locationId];
      GamePiece scoringGamePiece = kScoringLocationPiece[locationId];

      System.out.println("Running: Go to grid (id: " + locationId + ") from " + start);
      if (isRedAlliance) {
        scoringWaypoint = PathUtil.flip(scoringWaypoint);
      }

      Command moveToScoringWaypoint;
      if (kDynamicPathGenerationEnabled) {
        DynamicPathGenerator gen =
            new DynamicPathGenerator(start, scoringWaypoint, swerveSubsystem);
        moveToScoringWaypoint = gen.getCommand();
      } else
        moveToScoringWaypoint =
            PathGeneration.createDynamicAbsolutePath(
                start, scoringWaypoint, swerveSubsystem, kWaypointPathConstraints);

      BooleanSupplier isSelectedNodeCone = () -> scoringGamePiece.equals(GamePiece.CONE);
      Command runOuttake =
          new ConditionalCommand(
              new OuttakeCone(intakeSubsystem),
              new OuttakeCube(intakeSubsystem),
              isSelectedNodeCone);
      // Command stow = new StowArmElevator(elevatorSubsystem, armSubsystem);
      // Set arm and elevator command and end pose based on node type and height
      Pose2d scoringLocation;

      switch (gridScoreHeight.get()) {
        case HIGH:
          scoringLocation = kHighBlueScoringPoses[locationId];
          break;
        case MID:
          scoringLocation = kMidBlueScoringPoses[locationId];
          break;
        default:
          scoringLocation = kBottomBlueScoringPoses[locationId];
      }

      if (FeatureFlags.kIntakeAutoScoreDistanceSensorOffset && isSelectedNodeCone.getAsBoolean()) {
        double offset =
            isRedAlliance
                ? intakeSubsystem.getGamePieceOffset()
                : -intakeSubsystem.getGamePieceOffset();
        offset = MathUtil.clamp(offset, -Units.inchesToMeters(7), Units.inchesToMeters(7));
        System.out.println(
            "AUTO SCORE: Offsetting cone by " + offset + " meters based on distance sensors!");

        scoringLocation =
            scoringLocation.plus(new Transform2d(new Translation2d(0, offset), new Rotation2d()));
      }

      if (isRedAlliance) {
        scoringLocation = PathUtil.flip(scoringLocation);
      }

      Command moveToScoringLocation =
          PathGeneration.createDynamicAbsolutePath(
              scoringWaypoint, scoringLocation, swerveSubsystem, kPathToDestinationConstraints);

      Command successLEDs = new SetAllColor(ledSubsystem, kSuccess).withTimeout(2.5);

      Command errorLEDs = new SetAllColor(ledSubsystem, kError).withTimeout(2.5);
      Command runningLEDs =
          new ConditionalCommand(
              new SetAllBlink(ledSubsystem, kCone),
              new SetAllBlink(ledSubsystem, kCube),
              isSelectedNodeCone);

      Command stowArmElevator =
          new StowEndEffector(elevatorSubsystem, armSubsystem, isSelectedNodeCone);

      Command autoScore =
          Commands.sequence(
                  moveToScoringWaypoint,
                  Commands.parallel(moveToScoringLocation, moveArmElevatorToPreset.asProxy()),
                  Commands.either(
                      Commands.sequence(
                          runOuttake.asProxy(), stowArmElevator.asProxy().withTimeout(4)),
                      Commands.none(),
                      () -> kAutoOuttakeEnabled))
              .deadlineWith(runningLEDs.asProxy())
              .handleInterrupt(errorLEDs::schedule)
              .finallyDo(
                  (interrupted) -> {
                    if (!interrupted) successLEDs.schedule();
                  })
              .until(cancelCommand);

      addChildCommands(autoScore);
    }
    super.initialize();
  }
}
