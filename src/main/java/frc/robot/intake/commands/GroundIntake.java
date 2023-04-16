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
import frc.robot.arm.commands.KeepArm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class GroundIntake extends ParallelCommandGroup {
  public enum ConeOrientation {
    SITTING_CONE,
    STANDING_CONE,
  }

  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;
  private ConeOrientation coneOrientation = ConeOrientation.STANDING_CONE;

  private BooleanSupplier isCurrentPieceCone;

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      BooleanSupplier isCurrentPieceCone) {
    this(
        elevatorSubsystem,
        armSubsystem,
        intakeSubsystem,
        ConeOrientation.SITTING_CONE,
        isCurrentPieceCone);
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      ConeOrientation coneOrientation,
      BooleanSupplier isCurrentPieceCone) {

    this.coneOrientation = coneOrientation;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;

    ArmPreset coneArmPreset =
        coneOrientation == ConeOrientation.SITTING_CONE
            ? ArmPreset.SITTING_CONE_GROUND_INTAKE
            : ArmPreset.STANDING_CONE_GROUND_INTAKE;

    addCommands(
        new ConditionalCommand(
                new IntakeCone(intakeSubsystem),
                new IntakeCube(intakeSubsystem),
                isCurrentPieceCone)
            .andThen(
                Commands.deadline(
                        new StowEndEffector(elevatorSubsystem, armSubsystem, isCurrentPieceCone),
                        new ConditionalCommand(
                                new IntakeCone(intakeSubsystem).repeatedly(),
                                new InstantCommand(),
                                isCurrentPieceCone)
                            .asProxy())
                    .andThen(new LatchGamePiece(intakeSubsystem, isCurrentPieceCone))),
        new ConditionalCommand(
            new SetArmAngle(armSubsystem, coneArmPreset).andThen(new KeepArm(armSubsystem)),
            new SetArmAngle(armSubsystem, ArmPreset.CUBE_GROUND_INTAKE)
                .andThen(new KeepArm(armSubsystem)),
            isCurrentPieceCone));
    super.initialize();
  }
}
