package frc.robot.auto.helpers;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static TrajectoryFactory trajectoryFactory;


    public static SendableChooser<Command> getDefaultChooser() {


        autoChooser = new SendableChooser<>();
        return autoChooser;
    }

    public static Command getCommand() {
        return
            autoChooser.getSelected();
    }
}
