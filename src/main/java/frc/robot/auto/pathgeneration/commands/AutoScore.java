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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPosition;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPosition;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.ErrorBlinkingPattern;
import frc.robot.led.patterns.SuccessBlinkingPattern;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoScore extends CommandBase {
  public enum GridScoreHeight {
    HIGH,
    MID,
    LOW
  }

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;
  private GridScoreHeight gridScoreHeight;
  private BooleanSupplier isCurrentPieceCone;

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      GridScoreHeight gridScoreHeight,
      BooleanSupplier isCurrentPieceCone) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gridScoreHeight = gridScoreHeight;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    Pose2d src = swerveSubsystem.getPose();
    Pose2d sink = new Pose2d();
    Pose2d preSink = new Pose2d();

    int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
    if (locationId == -1) {
      System.out.println("locationId was invalid");
      if (ledSubsystem != null) {
        new LEDSetAllSectionsPattern(ledSubsystem, new ErrorBlinkingPattern()).schedule();
        return;
      }
    }

    switch (gridScoreHeight) {
      case HIGH:
        sink = kHighBlueScoringPoses[locationId];
      case MID:
        sink = kMidBlueScoringPoses[locationId];
      case LOW:
        sink = kBottomBlueScoringPoses[locationId];
    }
    preSink = kHighBlueScoringPoses[locationId];
    System.out.println("Running: Go to grid (id: " + locationId + ") from " + src);

    if (DriverStation.getAlliance() == Alliance.Red) {
      sink = PathUtil.flip(sink);
      preSink = PathUtil.flip(preSink);
    }

    // commands that will be run sequentially
    Command moveToPreSink = PathGeneration.createDynamicAbsolutePath(src, preSink, swerveSubsystem);
    Command moveArmElevatorToPreset = new InstantCommand();

    switch (gridScoreHeight) {
      case HIGH:
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CONE_HIGH),
                    new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.CUBE_HIGH),
                    isCurrentPieceCone),
                new ConditionalCommand(
                    new SetArmAngle(armSubsystem, ArmPosition.CONE_HIGH),
                    new SetArmAngle(armSubsystem, ArmPosition.CUBE_HIGH),
                    isCurrentPieceCone));
      case MID:
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.ANY_PIECE_MID),
                new ConditionalCommand(
                    new SetArmAngle(armSubsystem, ArmPosition.CONE_MID),
                    new SetArmAngle(armSubsystem, ArmPosition.CUBE_MID),
                    isCurrentPieceCone));
      case LOW:
        moveArmElevatorToPreset =
            new ParallelCommandGroup(
                new SetElevatorHeight(elevatorSubsystem, ElevatorPosition.ANY_PIECE_MID),
                new SetArmAngle(armSubsystem, ArmPosition.ANY_PIECE_LOW));
    }

    // TODO FINISHE command
    Command runIntake =
        new ConditionalCommand(
            new IntakeCube(intakeSubsystem), new IntakeCube(intakeSubsystem), isCurrentPieceCone);
    Command moveToScoringLocation =
        PathGeneration.createDynamicAbsolutePath(preSink, sink, swerveSubsystem);
    LEDSetAllSectionsPattern signalLED =
        new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern());

    // return sequential of all above commands
    Command finalCommand =
        Commands.sequence(
            moveToPreSink,
            Commands.deadline(
                runIntake.withTimeout(8), moveArmElevatorToPreset, moveToScoringLocation),
            signalLED);

    finalCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
