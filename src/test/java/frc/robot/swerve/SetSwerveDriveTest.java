package frc.robot.swerve;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.swerve.commands.TeleopSwerveFieldOriented;
import org.junit.*;

import org.junit.jupiter.api.Test;

public class SetSwerveDriveTest {
    public static final double DELTA = 0.05; // acceptable deviation range
    SwerveDrive swerveDrive;

    @Test
    public void TestTeleop() {
        /*
            Make the suppliers that are accepted by the TeleopSwerveFieldOriented command
         */
        Joystick driver = new Joystick(0);

        boolean fieldRelative = true;
        boolean openLoop = true;

    }


    private static void runScheduler(double seconds) {
        try {
            for (int i = 0; i < 10; ++i) {
                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
                CommandScheduler.getInstance().run();
                Thread.sleep((long) (seconds * 100));
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
