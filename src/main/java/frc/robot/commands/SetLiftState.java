// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;

public class SetLiftState extends ParallelCommandGroup {
	public enum LiftPreset {
		SCORE_CONE_HIGH(Arm.ArmPreset.CONE_HIGH, Elevator.ElevatorPreset.CONE_HIGH),
		SCORE_CONE_MID(Arm.ArmPreset.CONE_MID,Elevator.ElevatorPreset.ANY_PIECE_MID),
		SCORE_CONE_LOW(Arm.ArmPreset.ANY_PIECE_LOW,Elevator.ElevatorPreset.ANY_PIECE_LOW),
		DOUBLE_SUBSTATION_CONE(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE,Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
		STOW_CONE(Arm.ArmPreset.STOW_CONE,Elevator.ElevatorPreset.STOW_CONE),
		SCORE_CUBE_HIGH(Arm.ArmPreset.CUBE_HIGH,Elevator.ElevatorPreset.CUBE_HIGH),
		SCORE_CUBE_MID(Arm.ArmPreset.CUBE_MID,Elevator.ElevatorPreset.ANY_PIECE_MID),
		SCORE_CUBE_LOW(Arm.ArmPreset.ANY_PIECE_LOW,Elevator.ElevatorPreset.ANY_PIECE_LOW),
		DOUBLE_SUBSTATION_CUBE(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE,Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
		STOW_CUBE(Arm.ArmPreset.STOW_CUBE,Elevator.ElevatorPreset.STOW_CUBE);

		public final Arm.ArmPreset armPreset;
		public final Elevator.ElevatorPreset elevatorPreset;
		LiftPreset(Arm.ArmPreset armPreset, Elevator.ElevatorPreset elevatorPreset){
			this.armPreset=armPreset;
			this.elevatorPreset=elevatorPreset;
		}
	}

	public SetLiftState(
			Elevator elevatorSubsystem, Arm armSubsystem, LiftPreset liftPreset) {
		addCommands(
				new SetElevatorHeight(elevatorSubsystem, liftPreset.elevatorPreset),
				new SetArmAngle(armSubsystem,liftPreset.armPreset));
	}
	public SetLiftState(
			Elevator elevatorSubsystem,
			Arm armSubsystem,
			double elevatorExtension,
			Rotation2d armAngle
			) {
		addCommands(
				new SetElevatorHeight(elevatorSubsystem, elevatorExtension),
				new SetArmAngle(armSubsystem,armAngle));
	}
}
