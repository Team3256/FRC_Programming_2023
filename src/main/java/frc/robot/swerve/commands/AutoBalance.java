package frc.robot.swerve.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveDrive;

public class AutoBalance extends CommandBase {
    SwerveDrive swerveDrive;

    public AutoBalance(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }
    @Override
    public void execute() {
        if (swerveDrive.isTiltedForward()) {
            swerveDrive.drive(new Translation2d(-1, 0),0,true, true);
        } else if (swerveDrive.isTiltedBackward()) {
            swerveDrive.drive(new Translation2d(1, 0),0,true, true);
        } else {
            swerveDrive.drive(new Translation2d(0, 0),0,true, true);
        }
    }
    @Override
    public void initialize() {
    }
    @Override
    public void end(boolean interrupted) {
        swerveDrive.setAngleMotorsNeutralMode(NeutralMode.Brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}