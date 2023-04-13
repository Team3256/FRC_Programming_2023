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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.led.commands.SetAllColor;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoIntakeAtDoubleSubstation extends ParentCommand {
  public enum SubstationLocation {
    // From driver's POV
    RIGHT_SIDE,
    LEFT_SIDE
  }

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;
  private Supplier<SubstationLocation> substationLocation;
  private BooleanSupplier cancelCommand;
  private BooleanSupplier isAutoScoreMode;
  private BooleanSupplier isCurrentPieceCone;

  public AutoIntakeAtDoubleSubstation(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      Supplier<SubstationLocation> substationLocation,
      BooleanSupplier cancelCommand,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier isCurrentPieceCone) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.substationLocation = substationLocation;
    this.isAutoScoreMode = isAutoScoreMode;
    this.cancelCommand = cancelCommand;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    System.out.println(
        "Is running auto intake instead of presets: " + isAutoScoreMode.getAsBoolean());
    if (!isAutoScoreMode.getAsBoolean()) {
      System.out.println(
          "Running intake preset at double substation for cone? "
              + isCurrentPieceCone.getAsBoolean());
      new ConditionalCommand(
              new ParallelCommandGroup(
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CONE),
                  new IntakeCone(intakeSubsystem, ledSubsystem)),
              new ParallelCommandGroup(
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CUBE),
                  new IntakeCube(intakeSubsystem, ledSubsystem)),
              isCurrentPieceCone)
          .schedule();
      return;
    }

    System.out.println("Running: Go to substation from " + swerveSubsystem.getPose());
    Alliance alliance = DriverStation.getAlliance();

    Pose2d end;
    if (substationLocation.get().equals(SubstationLocation.RIGHT_SIDE)) {
      // Left and right are different depending on alliance
      if (alliance == Alliance.Red) {
        end = kBlueOuterDoubleSubstationPose;
      } else {
        end = kBlueInnerDoubleSubstationPose;
      }
    } else {
      if (alliance == Alliance.Red) {
        end = kBlueInnerDoubleSubstationPose;
      } else {
        end = kBlueOuterDoubleSubstationPose;
      }
    }

    Pose2d substationWaypoint =
        new Pose2d(
            end.getX() - kSubstationWaypointOffset,
            end.getY(),
            end.getRotation().plus(kArmFckConstant));

    if (alliance == Alliance.Red) {
      end = PathUtil.flip(end);
      substationWaypoint = PathUtil.flip(substationWaypoint);
    }

    // commands that will be run sequentially
    Command moveToWaypoint =
        PathGeneration.createDynamicAbsolutePath(
            swerveSubsystem.getPose(),
            substationWaypoint,
            swerveSubsystem,
            kWaypointPathConstraints);

    Command moveArmElevatorToPreset =
        new ParallelCommandGroup(
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CONE),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CUBE),
                isCurrentPieceCone));

    Command runIntake =
        new ConditionalCommand(
            new IntakeCone(intakeSubsystem, ledSubsystem),
            new IntakeCube(intakeSubsystem, ledSubsystem),
            isCurrentPieceCone);
    Command moveToSubstation =
        PathGeneration.createDynamicAbsolutePath(
            substationWaypoint, end, swerveSubsystem, kPathToDestinationConstraints);

    Command stowArmElevator =
        new StowEndEffector(elevatorSubsystem, armSubsystem, isCurrentPieceCone);

    Command runningLEDs =
        new ConditionalCommand(
            new SetAllColor(ledSubsystem, kCone),
            new SetAllColor(ledSubsystem, kCube),
            isCurrentPieceCone);
    Command successLEDs = new SetAllBlink(ledSubsystem, kSuccess);
    Command errorLEDs = new SetAllBlink(ledSubsystem, kError);

    // Automatically intake at the double substation
    Command autoIntakeCommand =
        Commands.sequence(
                moveToWaypoint,
                Commands.deadline(
                    runIntake.withTimeout(8), moveArmElevatorToPreset, moveToSubstation),
                stowArmElevator)
            .deadlineWith(runningLEDs.asProxy())
            .until(cancelCommand)
            .finallyDo(
                (interrupted) -> {
                  if (!interrupted) successLEDs.schedule();
                })
            .handleInterrupt(() -> errorLEDs.schedule());

    addChildCommands(autoIntakeCommand);
    super.initialize();
  }
}
