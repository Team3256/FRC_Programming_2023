package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    public StopShooter(ShooterSubsystem shooter) {
        this.shooterSubsystem = shooter;

        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
