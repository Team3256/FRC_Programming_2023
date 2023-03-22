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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.AutoMoveBlinkingPattern;
import frc.robot.led.patterns.ErrorBlinkingPattern;
import frc.robot.led.patterns.SuccessBlinkingPattern;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoIntakeAtSubstation extends CommandBase {
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
  private SubstationLocation substationLocation;
  private BooleanSupplier cancelCommand;
  private BooleanSupplier isCurrentPieceCone;

  public AutoIntakeAtSubstation(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      SubstationLocation substationLocation,
      BooleanSupplier cancelCommand,
      BooleanSupplier isCurrentPieceCone) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.substationLocation = substationLocation;
    this.cancelCommand = cancelCommand;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    System.out.println("Running: Go to substation from " + swerveSubsystem.getPose());
    Alliance alliance = DriverStation.getAlliance();

    Pose2d end;
    if (substationLocation.equals(SubstationLocation.RIGHT_SIDE)) {
      // Left and right are different depending on alliance
      if (alliance == Alliance.Red) {
        end = kBlueTopDoubleSubstationPose;
      } else {
        end = kBlueBottomDoubleSubstationPose;
      }
    } else {
      if (alliance == Alliance.Red) {
        end = kBlueBottomDoubleSubstationPose;
      } else {
        end = kBlueTopDoubleSubstationPose;
      }
    }

    Pose2d substationWaypoint =
        new Pose2d(end.getX() - kSubstationWaypointOffset, end.getY(), end.getRotation());

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
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPreset.DOUBLE_SUBSTATION),
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, Arm.ArmPreset.DOUBLE_SUBSTATION_CONE),
                new SetArmAngle(armSubsystem, Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE),
                isCurrentPieceCone));

    Command runIntake =
        new ConditionalCommand(
            new IntakeCone(intakeSubsystem, ledSubsystem),
            new IntakeCube(intakeSubsystem, ledSubsystem),
            isCurrentPieceCone);
    Command moveToSubstation =
        PathGeneration.createDynamicAbsolutePath(
            substationWaypoint, end, swerveSubsystem, kPathToDestinationConstraints);
    Command stopIntake = new IntakeOff(intakeSubsystem);
    Command stowArmElevator = new StowArmElevator(elevatorSubsystem, armSubsystem);
    Command moveAwayFromSubstation =
        PathGeneration.createDynamicAbsolutePath(
            end, substationWaypoint, swerveSubsystem, kPathToDestinationConstraints);

    Command runningLEDs = new LEDSetAllSectionsPattern(ledSubsystem, new AutoMoveBlinkingPattern());
    Command successLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern()).withTimeout(5);
    Command errorLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new ErrorBlinkingPattern()).withTimeout(5);

    Command autoIntakeCommand =
        Commands.sequence(
                moveToWaypoint,
                Commands.deadline(
                    runIntake.withTimeout(8), moveArmElevatorToPreset, moveToSubstation),
                Commands.deadline(moveAwayFromSubstation, stowArmElevator, stopIntake))
            .deadlineWith(runningLEDs)
            .until(cancelCommand)
            .finallyDo((interrupted) -> successLEDs.schedule())
            .handleInterrupt(() -> errorLEDs.schedule());

    autoIntakeCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
