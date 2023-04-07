// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;
import frc.robot.intake.Intake;
import frc.robot.limelight.Limelight;

public class GroundIntake extends CommandBase {

  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;

  public GroundIntake(Elevator elevatorSubsystem, Arm armSubsystem, Intake intakeSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // TODO: Add gamepiece detection
  @Override
  public void initialize() {
    Limelight.setPipelineIndex(
        FrontConstants.kLimelightNetworkTablesName, kClassifierPipelineIndex);

    Commands.parallel(
        new ZeroElevator(elevatorSubsystem),
        new SetArmAngle(armSubsystem, Arm.ArmPreset.GROUND_INTAKE),
        new IntakeCone(intakeSubsystem));
  }

  @Override
  public void end(boolean interrupted) {
    Limelight.setPipelineIndex(FrontConstants.kLimelightNetworkTablesName, 0);
  }
}
