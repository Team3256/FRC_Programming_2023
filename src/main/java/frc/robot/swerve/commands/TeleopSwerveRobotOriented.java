package frc.robot.swerve.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;


public class TeleopSwerveRobotOriented extends CommandBase {
    private final SwerveDrive drivetrainSubsystem;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public TeleopSwerveRobotOriented(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier,
                                            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    public TeleopSwerveRobotOriented(SwerveDrive drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        this.rotationSupplier = () -> 0;

        addRequirements(drivetrainSubsystem);

        SmartDashboard.putBoolean("IS Robot Oriented", false);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("IS Robot Oriented", true);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrainSubsystem.drive(
                new Translation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble()),
                rotationSupplier.getAsDouble(),
                true,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new Translation2d(0,0),0,true,true);
        SmartDashboard.putBoolean("IS Robot Oriented", false);
    }
}
