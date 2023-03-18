package frc.robot.mole.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mole.Mole;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.SuccessBlinkingPattern;

public class MoleIntakeCube extends CommandBase {

    private final Mole moleSubsystem;

    private LED ledSubsystem;

    public MoleIntakeCube(Mole moleSubsystem){
        this.moleSubsystem = moleSubsystem;
        addRequirements(moleSubsystem);
    }

    public MoleIntakeCube(Mole moleSubsystem, LED ledSubsystem){
        this.moleSubsystem = moleSubsystem;
        this.ledSubsystem = ledSubsystem;
        addRequirements(moleSubsystem);
    }

    @Override
    public void initialize(){
        moleSubsystem.intakeCube();
    }

    @Override
    public void end(boolean interrupted){
        moleSubsystem.off();
        if(!interrupted && ledSubsystem != null){
            new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern())
                    .withTimeout(3)
                    .schedule();
        }
    }

    @Override
    public boolean isFinished(){
        return moleSubsystem.isCurrentSpiking();
    }
}
