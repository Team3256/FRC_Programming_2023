package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferManualReverse extends CommandBase {

    TransferSubsystem transferSubsystem;

    /**
     * Make sure to set as not interruptable, so that auto index commands don't take control.
     * @param transferSubsystem Transfer Subsystem =
     */
     public TransferManualReverse(TransferSubsystem transferSubsystem){
         this.transferSubsystem = transferSubsystem;
         addRequirements(transferSubsystem);
     }

    @Override
    public void initialize() {
         transferSubsystem.manualReverse();

    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.off();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
