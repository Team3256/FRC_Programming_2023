package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

import static frc.robot.Constants.HangerConstants.CURRENT_THRESHOLD;

/*
* Same as HangerZero but doesnt set encoders to 0, instead just syncs the motors on the bar
 */
public class HangerSyncOnBar extends CommandBase {
    private HangerSubsystem hanger;

    private double leftCurrentThreshold = CURRENT_THRESHOLD;
    private double rightCurrentThreshold = CURRENT_THRESHOLD;

    private boolean leftMotorRunning = true;
    private boolean rightMotorRunning = true;

    public HangerSyncOnBar(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }

    public HangerSyncOnBar(HangerSubsystem hanger, double leftCurrentThreshold, double rightCurrentThreshold) {
        this.hanger = hanger;
        this.leftCurrentThreshold = leftCurrentThreshold;
        this.rightCurrentThreshold = rightCurrentThreshold;
        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftMotorRunning = true;
        rightMotorRunning = true;

        hanger.retractLeftContinuouslyToZero();
        hanger.retractRightContinuouslyToZero();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (leftMotorRunning && hanger.isLeftHangerCurrentSpiking(leftCurrentThreshold)){
            hanger.stopLeftMotor();
            leftMotorRunning = false;
        }

        if (rightMotorRunning && hanger.isRightHangerCurrentSpiking(rightCurrentThreshold)){
            hanger.stopRightMotor();
            rightMotorRunning = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Follows the Motors, so they don't get out of sync
        hanger.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !rightMotorRunning && !leftMotorRunning;
    }

}