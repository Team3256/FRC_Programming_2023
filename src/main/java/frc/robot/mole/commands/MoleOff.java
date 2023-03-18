package frc.robot.mole.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;

public class MoleOff extends CommandBase {
    private final Mole moleSubsystem;

    public MoleOff(Mole moleSubsystem){
        this.moleSubsystem = moleSubsystem;
        addRequirements(moleSubsystem);
    }

    @Override
    public void initialize(){
        moleSubsystem.off();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
