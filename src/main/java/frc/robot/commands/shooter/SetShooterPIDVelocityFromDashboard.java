package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.ShooterSubsystem;


public class SetShooterPIDVelocityFromDashboard extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private ShooterSubsystem shooterSubsystem;

    public SetShooterPIDVelocityFromDashboard(ShooterSubsystem shooter) {
        this.shooterSubsystem = shooter;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
    }

    public  SetShooterPIDVelocityFromDashboard(ShooterSubsystem flywheelSubsystem, XboxController operatorController) {
        this(flywheelSubsystem);
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenHeld(new WaitAndVibrateCommand(operatorController, 0.05));
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Custom Velocity", 1200);
        SmartDashboard.setDefaultNumber("Custom Hood Angle", 0);
    }

    @Override
    public void execute() {
        double velocity = SmartDashboard.getNumber("Custom Velocity", 0);
        double hoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0);
        SmartDashboard.putNumber("Limelight", Limelight.getRawDistanceToTarget());

        double pidOutput = 0;
        if (velocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), velocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), velocity);
        }

        shooterSubsystem.setVelocityPID(velocity, pidOutput);
        shooterSubsystem.setHoodAngle(hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
    }
}
