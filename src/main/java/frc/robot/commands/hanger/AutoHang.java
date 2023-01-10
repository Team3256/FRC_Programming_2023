package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.HangerConstants.*;

public class AutoHang extends SequentialCommandGroup {
    HangerSubsystem hanger;
    public AutoHang(HangerSubsystem hanger, IntakeSubsystem intake) {
        this.hanger = hanger;
        addCommands(
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT),
                new HangerPartial(hanger),
                new WaitCommand(PARTIAL_EXTEND_WAIT),
                new HangerPneumaticSlant(hanger, intake),
                new HangerExtend(hanger),
                new WaitCommand(EXTEND_WAIT),
                new HangerPneumaticUpright(hanger),
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT),
                new HangerPartial(hanger),
                new WaitCommand(PARTIAL_EXTEND_WAIT),
                new HangerPneumaticSlant(hanger, intake),
                new HangerExtend(hanger),
                new WaitCommand(EXTEND_WAIT),
                new HangerPneumaticUpright(hanger),
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT)
        );
    }

}
