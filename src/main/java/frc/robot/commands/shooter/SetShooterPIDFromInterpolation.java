package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class SetShooterPIDFromInterpolation extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;

    private double targetVelocity = 0;
    private double targetHoodAngle = 0;
    private double pidOutput = 0;

    private double currentDistance = 0;

    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier estimatedDistance;
    private BooleanSupplier isTargetDetected;

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem, DoubleSupplier estimatedDistance, BooleanSupplier isTargetDetected) {
        this.shooterSubsystem = flywheelSubsystem;
        this.isTargetDetected = isTargetDetected;
        this.estimatedDistance = estimatedDistance;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
    }

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem, DoubleSupplier odomDistanceToTarget, BooleanSupplier isTargetDetected, XboxController operatorController) {
        this(flywheelSubsystem, odomDistanceToTarget, isTargetDetected);
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
    }

    @Override
    public void initialize() {
        System.out.println("Velocity PID Ramping Up");
        Limelight.enable();
    }

    @Override
    public void execute() {
        if(isTargetDetected.getAsBoolean()){
            currentDistance = Limelight.getRawDistanceToTarget();
        }
        else {
            currentDistance = estimatedDistance.getAsDouble();
        }
        
        targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(currentDistance) + 15; // PID bad ig
        targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(currentDistance);

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Interpolation Target Velocity", targetVelocity);
            SmartDashboard.putNumber("Interpolation Target Hood Angle", targetHoodAngle);

        }

        if (targetVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        }

        shooterSubsystem.setVelocityPID(targetVelocity, pidOutput);
        shooterSubsystem.setHoodAngle(targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
//        Limelight.disable();
    }

}
