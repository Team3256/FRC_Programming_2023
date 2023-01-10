package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ResetPoseCommand extends CommandBase {

    SwerveDrive swerveDrive;

    /**
     * Command for scheduling reseting the pose of the robot.
     * @param drivetrainSubsystem drivetrain instance
     */
    public ResetPoseCommand (SwerveDrive drivetrainSubsystem) {
        this.swerveDrive = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        swerveDrive.resetOdometry(new Pose2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
    }
}

