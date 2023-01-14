package frc.robot.swerve.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TeleopSwerveFieldOriented extends CommandBase {

    private final SwerveDrive drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public TeleopSwerveFieldOriented(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier,
                                            DoubleSupplier rotationSupplier) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;


        addRequirements(drivetrainSubsystem);
    }

    public TeleopSwerveFieldOriented(SwerveDrive drivetrainSubsystem) { // constructor that sets values to 0
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        this.rotationSupplier = () -> 0;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing swerve");
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        Translation2d translation = new Translation2d(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble());

        drivetrainSubsystem.drive(
                translation,
                rotationSupplier.getAsDouble(),
                true,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        Translation2d translation = new Translation2d(
                0,
                0
        );
        drivetrainSubsystem.drive(
                translation,
                0,
                true,
                true
        );
    }
}