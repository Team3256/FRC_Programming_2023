package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroHoodMotorCommand extends CommandBase {
    private RobotLogger logger = new RobotLogger(ZeroHoodMotorCommand.class.getCanonicalName());

    private final ShooterSubsystem shooterSubsystem;

    /**
     * @param subsystem
     * zeros the hood motor and it's sensor
     */
    public ZeroHoodMotorCommand(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logger.info("Zeroing Hood Motor");
        shooterSubsystem.hoodSlowReverse();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopHood();
        shooterSubsystem.zeroHoodMotorSensor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooterSubsystem.isHoodLimitSwitchPressed();
    }
}
