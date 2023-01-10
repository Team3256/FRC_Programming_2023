package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

import java.math.BigDecimal;
import java.util.function.Supplier;


public class SetShooterPIDVelocityFromState extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;

    private ShooterSubsystem shooterSubsystem;
    private Supplier<ShooterState> shooterStateSupplier;

    private double pidOutput = 0;

    public SetShooterPIDVelocityFromState(ShooterSubsystem shooter, Supplier<ShooterState> shooterStateSupplier) {
        this.shooterSubsystem = shooter;
        this.shooterStateSupplier = shooterStateSupplier;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
       }

   public SetShooterPIDVelocityFromState(ShooterSubsystem flywheelSubsystem, Supplier<ShooterState> shooterStateSupplier, XboxController operatorController) {
        this(flywheelSubsystem, shooterStateSupplier);
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
   }

    @Override
    public void initialize() {

        System.out.println("Velocity PID Ramping Up");
    }

    @Override
    public void execute() {
        if (shooterStateSupplier.get().rpmVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), shooterStateSupplier.get().rpmVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), shooterStateSupplier.get().rpmVelocity);
        }

        shooterSubsystem.setHoodAngle(shooterStateSupplier.get().hoodAngle);
        shooterSubsystem.setVelocityPID(shooterStateSupplier.get().rpmVelocity, pidOutput);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
    }

}
