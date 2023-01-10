package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.HangerConstants.PNEUMATIC_WAIT_DURATION;

public class HangerTogglePneumatics extends CommandBase {
    private static final RobotLogger logger = new RobotLogger(HangerTogglePneumatics.class.getCanonicalName());

    private HangerSubsystem hanger;
    private IntakeSubsystem intake;

    public HangerTogglePneumatics(HangerSubsystem hanger, IntakeSubsystem intake) {
        this.hanger = hanger;
        this.intake = intake;
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hanger.pneumaticSlant();
        if (intake != null){
            intake.extend();
        } else {
            logger.info("Trying to Slant Hanger without Intake being initialized! Might lead to collisions");
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hanger.pneumaticUpright();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
