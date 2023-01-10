package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class OuttakeFast extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public OuttakeFast(TransferSubsystem transferSubsystem, IntakeSubsystem intakeSubsystem) {
        this.transferSubsystem = transferSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        transferSubsystem.outtake();
        intakeSubsystem.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.off();
        intakeSubsystem.off();
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
