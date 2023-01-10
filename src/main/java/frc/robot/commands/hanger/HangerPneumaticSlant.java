package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.HangerConstants.PNEUMATIC_WAIT_DURATION;

public class HangerPneumaticSlant extends CommandBase {
    private HangerSubsystem hanger;
    private IntakeSubsystem intake;
    protected Timer timer = new Timer();
    public HangerPneumaticSlant(HangerSubsystem hanger, IntakeSubsystem intake) {
        this.hanger = hanger;
        this.intake = intake;

        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intake.intakeDown();

        hanger.pneumaticSlant();

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