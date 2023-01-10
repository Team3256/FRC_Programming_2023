package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommandFieldOriented extends CommandBase {
    private RobotLogger logger = new RobotLogger(DefaultDriveCommandFieldOriented.class.getCanonicalName());
    private final SwerveDrive drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommandFieldOriented(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier,
                                            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    public DefaultDriveCommandFieldOriented(SwerveDrive drivetrainSubsystem) { // constructor that sets values to 0
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        this.rotationSupplier = () -> 0;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        logger.info("Field Oriented Drive Enabled");
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrainSubsystem.drive(
                 ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        rotationSupplier.getAsDouble(),
                        drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}