package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class HangerRetractForHang extends CommandBase {
    private HangerSubsystem hanger;

    public HangerRetractForHang(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }

    @Override
    public void initialize() {
        hanger.retractToHang();
    }

    @Override
    public void end(boolean interrupted) {
        hanger.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
