package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class HangerExtend extends CommandBase {
    private HangerSubsystem hanger;
    public HangerExtend(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hanger.extendToHangPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hanger.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hanger.isFullPositionReached();
    }
}