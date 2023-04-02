package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ZeroElevator;

public class IntakeFromGround extends SequentialCommandGroup {
    //Move the elevator and arm to its lowest position 
    //Then, apply current to slightly push the intake into the ground

    public IntakeFromGround(Elevator elevatorSubsystem, Arm armSubsystem) {
        addCommands(
            new WaitCommand(0),
            new ZeroElevator(elevatorSubsystem),
            new SetArmAngle(armSubsystem, Arm.ArmPreset.ANY_PIECE_LOW));
    }
}
