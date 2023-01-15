package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;

public class ElevatorLowPosition extends CommandBase {

    private final Elevator elevator;

    public ElevatorLowPosition(Elevator subsystem) {
        elevator = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elevator.setLowPosition();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        elevator.off();
    }
}
