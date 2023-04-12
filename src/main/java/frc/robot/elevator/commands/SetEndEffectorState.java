// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;

public class SetEndEffectorState extends ParallelCommandGroup {
  public enum EndEffectorPreset {
    SCORE_CONE_HIGH(Arm.ArmPreset.CONE_HIGH, Elevator.ElevatorPreset.CONE_HIGH),
    SCORE_CONE_MID(Arm.ArmPreset.CONE_MID, Elevator.ElevatorPreset.ANY_PIECE_MID),
    SCORE_ANY_LOW(Arm.ArmPreset.ANY_PIECE_LOW, Elevator.ElevatorPreset.ANY_PIECE_LOW),
    DOUBLE_SUBSTATION_CONE(
        Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
    STOW_CONE(Arm.ArmPreset.STOW_CONE, Elevator.ElevatorPreset.STOW_CONE),
    SCORE_CUBE_HIGH(Arm.ArmPreset.CUBE_HIGH, Elevator.ElevatorPreset.CUBE_HIGH),
    SCORE_CUBE_MID(Arm.ArmPreset.CUBE_MID, Elevator.ElevatorPreset.ANY_PIECE_MID),
    DOUBLE_SUBSTATION_CUBE(
        Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
    STOW_CUBE(Arm.ArmPreset.STOW_CUBE, Elevator.ElevatorPreset.STOW_CUBE);

    public final Arm.ArmPreset armPreset;
    public final Elevator.ElevatorPreset elevatorPreset;

    EndEffectorPreset(Arm.ArmPreset armPreset, Elevator.ElevatorPreset elevatorPreset) {
      this.armPreset = armPreset;
      this.elevatorPreset = elevatorPreset;
    }
  }

  public SetEndEffectorState(
      Elevator elevatorSubsystem, Arm armSubsystem, EndEffectorPreset endEffectorPreset) {
    addCommands(
        new SetElevatorExtension(elevatorSubsystem, endEffectorPreset.elevatorPreset)
            .beforeStarting(new WaitUntilCommand(armSubsystem::isSafeFromElevator)),
        new SetArmAngle(armSubsystem, endEffectorPreset.armPreset));
  }

  public SetEndEffectorState(
      Elevator elevatorSubsystem, Arm armSubsystem, double elevatorExtension, Rotation2d armAngle) {
    addCommands(
        new SetElevatorExtension(elevatorSubsystem, elevatorExtension)
            .beforeStarting(new WaitUntilCommand(armSubsystem::isSafeFromElevator)),
        new SetArmAngle(armSubsystem, armAngle));
  }
}
