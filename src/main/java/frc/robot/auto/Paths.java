package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.AutoAlignDriveCommand;
// import frc.robot.hardware.Limelight;
import frc.robot.helper.auto.AutoCommandMarker;
import frc.robot.helper.auto.AutoCommandRunner;
import frc.robot.swerve.SwerveDrive;




import java.util.List;




public class Paths {
    private static TrajectoryFactory trajectoryFactory;


    public static void initialize(SwerveDrive drive) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;

    }
}








