package frc.robot.mole.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants.*;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConstants.*;
import frc.robot.arm.commands.*;
import frc.robot.elevator.commands.*;
import frc.robot.intake.Intake;
import frc.robot.mole.Mole;

public class GroundIntake extends SequentialCommandGroup {
    private Mole moleSubsystem;

    public GroundIntake(Elevator elevatorSubsystem, Arm armSubsystem, Intake intakeSubsystem) {
        addCommands(
            new ZeroElevator(elevatorSubsystem), 
            new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(-35)),
            // new some intake command
            );
    }
}
