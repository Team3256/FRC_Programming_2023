package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climb.Climb;

public class RetractClimb extends CommandBase {
    private final Climb climbSubsystem;

    public RetractClimb(Climb climbSubsystem){
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
        climbSubsystem.retractClimb();
    }

    @Override
    public void end(boolean interrupted){
        climbSubsystem.off();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
