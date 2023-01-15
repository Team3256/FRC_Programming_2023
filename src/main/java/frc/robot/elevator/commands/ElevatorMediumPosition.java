package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class ElevatorMediumPosition extends CommandBase {

    private final Elevator elevator;

    public ElevatorMediumPosition(Elevator subsystem) {
        elevator = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elevator.setMediumPosition();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        elevator.off();
    }
}
