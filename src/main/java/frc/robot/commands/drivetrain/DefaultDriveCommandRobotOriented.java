package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommandRobotOriented extends CommandBase {
    private RobotLogger logger = new RobotLogger(DefaultDriveCommandRobotOriented.class.getCanonicalName());

    private final SwerveDrive drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommandRobotOriented(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier,
                                            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    public DefaultDriveCommandRobotOriented(SwerveDrive drivetrainSubsystem) { // constructor that sets values to 0
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        this.rotationSupplier = () -> 0;

        addRequirements(drivetrainSubsystem);

        SmartDashboard.putBoolean("IS Robot Oriented", false);
    }

    @Override
    public void initialize() {
        logger.info("Robot Oriented Drive Enabled");
        SmartDashboard.putBoolean("IS Robot Oriented", true);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        rotationSupplier.getAsDouble()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        SmartDashboard.putBoolean("IS Robot Oriented", false);
    }
}