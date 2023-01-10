// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transfer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferOff extends CommandBase {

    private final TransferSubsystem transferSubsystem;

    public TransferOff(TransferSubsystem transferSubsystem) {
        this.transferSubsystem = transferSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(transferSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        transferSubsystem.off();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}