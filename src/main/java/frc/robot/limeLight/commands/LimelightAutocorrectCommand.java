package frc.robot.limelight.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimeLight;
import frc.robot.helper.Polynomial;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.swerve.helpers.SwerveModule;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import static frc.robot.Constants.LimelightAutoCorrectConstants.*;

public class LimelightAutocorrectCommand extends CommandBase {
    private WeightedObservedPoints data;
    private final SwerveDrive robotDrive;
    private int counter;
    private final int initDistance;
    private int curDistance;

    private final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(POLYNOMIAL_DEGREE);

    /**
     * @param robotDrive drivetrain to move forward
     * @param initDistance current actual distance to goal
     * Plot data points of limelight error
     * Create correction function that brings error to 0
     */
    public LimelightAutocorrectCommand(SwerveDrive robotDrive, int initDistance){
        this.robotDrive = robotDrive;
        this.initDistance = initDistance;
    }

    @Override
    public void initialize() {
        counter = 0;
        data = new WeightedObservedPoints();
        curDistance = initDistance;
        Limelight.enable();
    }

    @Override
    public void execute(){
        if (!CommandScheduler.getInstance().isScheduled()) {
            CommandScheduler.getInstance().schedule(new LimelightAutocorrectStepCommand(robotDrive, data, curDistance));
            counter++;
            curDistance-=PACE_SIZE;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Polynomial result = new Polynomial(fitter.fit(data.toList()));
        SmartDashboard.putString("Limelight Distance Tuning Function Output", result.toString());
    }

    @Override
    public boolean isFinished(){
        return counter >= PACES;
    }
}