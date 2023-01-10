package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hardware.Limelight;
import frc.robot.helper.shooter.ShootingWhileMovingHelper;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveConstants.*;

public class SetShooterWhileMoving extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private PIDController alphaController;
    private ShootingWhileMovingHelper shootingWhileMovingHelper;

    private double targetVelocity = 0;
    private double targetHoodAngle = 0;
    private double targetAngle = 0;
    private double alpha = 0.1;
    private double pidOutput = 0;

    private ShooterSubsystem shooterSubsystem;
    private SwerveDrive swerveDrive;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    public SetShooterWhileMoving(SwerveDrive swerveDrive, ShooterSubsystem shooterSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.shooterSubsystem = shooterSubsystem;
        this.swerveDrive = swerveDrive;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
        alphaController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);

        addRequirements(swerveDrive, shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Velocity PID Ramping Up");
        Limelight.enable();

        this.shootingWhileMovingHelper = new ShootingWhileMovingHelper(
                shooterSubsystem,
                () -> swerveDrive.getEstimatedDistance(),
                () -> swerveDrive.getVelocity().getX(),
                () -> swerveDrive.getVelocity().getY()
        );
    }

    @Override
    public void execute() {
        ShootingWhileMovingHelper.ShootingWhileMovingState state = this.shootingWhileMovingHelper.calculate(alpha);

        alpha = state.alpha;
        targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(state.distance);
        targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(state.distance);

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Interpolation Target Velocity", targetVelocity);
            SmartDashboard.putNumber("Interpolation Target Hood Angle", targetHoodAngle);
            SmartDashboard.putNumber("Shooting while moving: Alpha", state.alpha);
            SmartDashboard.putNumber("Shooting while moving: Distance", state.distance);
            SmartDashboard.putBoolean("Shooting while moving: Ready", state.readyToShoot);
        }

        if (targetVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        }

        shooterSubsystem.setHoodAngle(targetHoodAngle);
        shooterSubsystem.setVelocityPID(targetVelocity, pidOutput);

        // thanks to dylan for this code <3
        double alphaPID = alphaController.calculate(swerveDrive.getEstimatedThetaOffset(), alpha);
        double speedSquared = Math.pow(translationXSupplier.getAsDouble(), 2) + Math.pow(translationYSupplier.getAsDouble(),2);
        double rotationalVelocity = speedSquared > Math.pow(0.1, 2) ?
                alphaPID :
                alphaPID + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, alphaPID);

        swerveDrive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        rotationalVelocity,
                        swerveDrive.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
        Limelight.disable();
    }

}
