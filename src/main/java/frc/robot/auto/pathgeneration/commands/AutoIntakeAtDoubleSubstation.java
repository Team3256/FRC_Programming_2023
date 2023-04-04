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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.led.patterns.Blink.ErrorPatternBlink;
import frc.robot.led.patterns.Blink.SuccessPatternBlink;
import frc.robot.led.patterns.ConePattern;
import frc.robot.led.patterns.CubePattern;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoIntakeAtDoubleSubstation extends CommandBase {
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
                  new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE)
                      .beforeStarting(new WaitCommand(0.3)),
                  new SetArmAngle(armSubsystem, Arm.ArmPreset.DOUBLE_SUBSTATION_CONE),
                  new IntakeCone(intakeSubsystem, ledSubsystem)),
              new ParallelCommandGroup(
                  new SetElevatorHeight(
                          elevatorSubsystem, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE)
                      .beforeStarting(new WaitCommand(0.3)),
                  new SetArmAngle(armSubsystem, Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE),
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
                new SetElevatorHeight(
                    elevatorSubsystem, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
                new SetElevatorHeight(
                    elevatorSubsystem, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
                isCurrentPieceCone),
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
    Command stowArmElevator = new StowArmElevator(elevatorSubsystem, armSubsystem, 0, 1);
    Command moveAwayFromSubstation =
        PathGeneration.createDynamicAbsolutePath(
            end, substationWaypoint, swerveSubsystem, kPathToDestinationConstraints);

    Command runningLEDs =
        new ConditionalCommand(
            new LEDSetAllSectionsPattern(ledSubsystem, new ConePattern()),
            new LEDSetAllSectionsPattern(ledSubsystem, new CubePattern()),
            isCurrentPieceCone);
    Command successLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new SuccessPatternBlink()).withTimeout(5);
    Command errorLEDs =
        new LEDSetAllSectionsPattern(ledSubsystem, new ErrorPatternBlink()).withTimeout(5);

    Command autoIntakeCommand =
        Commands.sequence(
                moveToWaypoint,
                Commands.deadline(
                    runIntake.withTimeout(8), moveArmElevatorToPreset, moveToSubstation),
                Commands.deadline(moveAwayFromSubstation, stowArmElevator, stopIntake))
            .deadlineWith(runningLEDs.asProxy())
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
