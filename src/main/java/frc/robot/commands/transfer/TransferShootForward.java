package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

import java.util.function.BooleanSupplier;

public class TransferShootForward extends CommandBase {
    private RobotLogger logger = new RobotLogger(TransferSubsystem.class.getCanonicalName());

    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    private double lastTimeReady = 0;
    private boolean ready = false;
    private BooleanSupplier override = () -> false;

    /**
     * If manually controlled, set as not interruptable, so that auto index commands don't take control.
     * @param transferSubsystem Transfer Subsystem
     */
    public TransferShootForward(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        this.transferSubsystem = transferSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(transferSubsystem);
    }

    public TransferShootForward(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, BooleanSupplier override) {
        this.transferSubsystem = transferSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.override = override;

        addRequirements(transferSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Ready to Shoot", shooterSubsystem.isAtSetPoint());
        
        // if (shooterSubsystem.isAtSetPoint() || override.getAsBoolean()) {
        //     if (override.getAsBoolean() && shooterSubsystem.isAtSetPoint()) {
        //         logger.warning("Overriding Shooting because of timeout override");
        //     }
//            if (!ready) lastTimeReady = Timer.getFPGATimestamp();
//            if (Timer.getFPGATimestamp() - lastTimeReady >= 0.15 && ready)
        transferSubsystem.forwardShoot();
//            ready = true;
//         } else {
// //            ready = false;
//             transferSubsystem.off();
//         }
    }

    @Override
    public void initialize() {
        transferSubsystem.setShooting(true);
        lastTimeReady = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.off();
        transferSubsystem.setShooting(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
