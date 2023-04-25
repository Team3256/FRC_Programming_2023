// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.elevator.commands.SetEndEffectorState.EndEffectorPreset;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class GroundIntake extends DebugCommandBase {

  private EndEffectorPreset coneEndEffectorPreset;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;
  private BooleanSupplier isCurrentPieceCone;

  public enum ConeOrientation {
    SITTING_CONE,
    STANDING_CONE,
  }

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      ConeOrientation coneOrientation,
      BooleanSupplier isCurrentPieceCone) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
    this.coneEndEffectorPreset =
        coneOrientation == ConeOrientation.SITTING_CONE
            ? EndEffectorPreset.SITTING_CONE_GROUND_INTAKE
            : EndEffectorPreset.STANDING_CONE_GROUND_INTAKE;

    addRequirements(elevatorSubsystem, armSubsystem, intakeSubsystem);
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

  @Override
  public void initialize() {
    super.initialize();
    Command setArmElevatorToPreset =
        Commands.either(
            new SetEndEffectorState(elevatorSubsystem, armSubsystem, coneEndEffectorPreset),
            new SetEndEffectorState(
                elevatorSubsystem, armSubsystem, EndEffectorPreset.CUBE_GROUND_INTAKE),
            isCurrentPieceCone);

    Command intakeAndStow =
        Commands.either(
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

    Commands.sequence(setArmElevatorToPreset, intakeAndStow.asProxy()).schedule();
  }
}
