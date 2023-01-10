package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.helper.CANTest;

import java.util.Set;

public class TestCANCommand extends CommandBase {

    @Override
    public void initialize() {
       CANTest.test();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
