package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

import static frc.robot.Constants.HangerConstants.PNEUMATIC_WAIT_DURATION;

public class HangerPneumaticUpright extends CommandBase {
    private HangerSubsystem hanger;
    protected Timer timer = new Timer();
    public HangerPneumaticUpright(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        hanger.pneumaticUpright();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(PNEUMATIC_WAIT_DURATION);
    }

}