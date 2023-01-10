// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transfer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferIndexForward extends CommandBase {

    private final TransferSubsystem transferSubsystem;

    /**
     * If manually controlled, set as not interruptable, so that auto index commands don't take control.
     * @param transferSubsystem Transfer Subsystem
     */
    public TransferIndexForward(TransferSubsystem transferSubsystem) {
        this.transferSubsystem = transferSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(transferSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        transferSubsystem.forward();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            System.out.println("Interrupted Transfer Forward");
        transferSubsystem.off();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}