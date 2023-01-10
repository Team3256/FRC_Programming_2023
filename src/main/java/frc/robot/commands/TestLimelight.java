package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;

public class TestLimelight extends CommandBase {

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("distance to target:", Limelight.getRawDistanceToTarget());
        SmartDashboard.putNumber("(tuned) distance to target:", Limelight.getRawDistanceToTarget());
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
