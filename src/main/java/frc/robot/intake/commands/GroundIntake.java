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
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import frc.robot.limelight.Limelight;
import java.util.function.BooleanSupplier;

public class GroundIntake extends ParentCommand {

  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;

  private BooleanSupplier isCurrentPieceCone;

  public GroundIntake(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      BooleanSupplier isCurrentPieceCone) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    if (kGamePieceDetection) {
      Limelight.setPipelineIndex(
          FrontConstants.kLimelightNetworkTablesName, kDetectorPipelineIndex);
      isCurrentPieceCone =
          () -> Limelight.isConeDetected(FrontConstants.kLimelightNetworkTablesName);
    }

    addChildCommands(
        new ZeroElevator(elevatorSubsystem),
        new ConditionalCommand(
            new SetArmAngle(armSubsystem, ArmPreset.CONE_GROUND_INTAKE),
            new SetArmAngle(armSubsystem, ArmPreset.CUBE_GROUND_INTAKE),
            isCurrentPieceCone),
        new ConditionalCommand(
            new IntakeCone(intakeSubsystem), new IntakeCube(intakeSubsystem), isCurrentPieceCone));

    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    Limelight.setPipelineIndex(FrontConstants.kLimelightNetworkTablesName, kDefaultPipeline);
    super.end(interrupted);
  }
}
