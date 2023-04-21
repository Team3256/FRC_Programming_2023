// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.Constants.FeatureFlags.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorExtension;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class GroundIntake extends ParallelCommandGroup {
  private boolean autoOptimize = false;

  public enum ConeOrientation {
    SITTING_CONE,
    STANDING_CONE,
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      BooleanSupplier isCurrentPieceCone) {
    this(
        elevatorSubsystem,
        armSubsystem,
        intakeSubsystem,
        ConeOrientation.STANDING_CONE,
        isCurrentPieceCone);
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      BooleanSupplier isCurrentPieceCone,
      ConeOrientation coneOrientation,
      boolean autoOptimize) {
    this(
        elevatorSubsystem,
        armSubsystem,
        intakeSubsystem,
        ConeOrientation.STANDING_CONE,
        isCurrentPieceCone);
    this.autoOptimize = autoOptimize;
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      BooleanSupplier isCurrentPieceCone,
      boolean autoOptimize) {
    this(
        elevatorSubsystem,
        armSubsystem,
        intakeSubsystem,
        ConeOrientation.STANDING_CONE,
        isCurrentPieceCone);
    this.autoOptimize = autoOptimize;
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      ConeOrientation coneOrientation,
      BooleanSupplier isCurrentPieceCone) {
    ArmPreset coneArmPreset = coneOrientation == ConeOrientation.SITTING_CONE
        ? ArmPreset.SITTING_CONE_GROUND_INTAKE
        : ArmPreset.STANDING_CONE_GROUND_INTAKE;

    Command runArmEarly = Commands.none();
    if (autoOptimize
        && (!isCurrentPieceCone.getAsBoolean()
            || (isCurrentPieceCone.getAsBoolean()
                && coneArmPreset == ArmPreset.SITTING_CONE_GROUND_INTAKE))) {
      runArmEarly = new SetArmAngle(armSubsystem, ArmPreset.STANDING_CONE_GROUND_INTAKE);
    }

    Command setArmElevatorToPreset = Commands.sequence(
        Commands.deadline(
            new SetElevatorExtension(elevatorSubsystem, Elevator.ElevatorPreset.GROUND_INTAKE)
                .asProxy(),
            runArmEarly.asProxy()),
        Commands.parallel(
            new ConditionalCommand(
                new SetArmAngle(armSubsystem, coneArmPreset),
                new SetArmAngle(armSubsystem, ArmPreset.CUBE_GROUND_INTAKE),
                isCurrentPieceCone)
                .asProxy(),
            new ZeroElevator(elevatorSubsystem).asProxy()));

    Command intakeAndStow = new ConditionalCommand(
        new IntakeCone(intakeSubsystem),
        new IntakeCube(intakeSubsystem),
        isCurrentPieceCone)
        .andThen(
            Commands.deadline(
                new StowEndEffector(elevatorSubsystem, armSubsystem, isCurrentPieceCone)
                    .asProxy(),
                new ConditionalCommand(
                    new IntakeCone(intakeSubsystem).repeatedly(),
                    new InstantCommand(),
                    isCurrentPieceCone)
                    .asProxy()));
    // .andThen(new LatchGamePiece(intakeSubsystem, isCurrentPieceCone))),

    addCommands(setArmElevatorToPreset, intakeAndStow.asProxy());
  }
}
