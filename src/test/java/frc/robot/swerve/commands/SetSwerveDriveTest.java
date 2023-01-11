package frc.robot.swerve.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.function.DoubleSupplier;

public class SetSwerveDriveTest {

    public static final double DELTA = 0.5; // acceptable deviation range
    public static SwerveDrive swerveDrive;

    @BeforeAll
    public static void setup() {
        assert HAL.initialize(2000, 0); // initialize the HAL, crash if failed
        CommandScheduler.getInstance().enable();
        DriverStationSim.setEnabled(true);
    }
/*

    Command defaultDriveCommand = new TeleopSwerveFieldOriented(
            swerveDrive,
            () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );

 */
    @Test
    public void TestingTest() {
        Assertions.assertEquals(true, true);
    }

    @Test
    public void TestVelocities() {
        swerveDrive = new SwerveDrive();

        DoubleSupplier
                leftY = () -> 0.0,
                leftX = () -> 0.9 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotation = () -> 0.0;

        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
        runScheduler(0.5);

        double[] testArray = {0, 0, 0, 0};
        Assertions.assertArrayEquals(testArray, swerveDrive.getModuleVelocity());
    }

//    @Test
    public void TestTeleopX() {

        swerveDrive = new SwerveDrive();

        DoubleSupplier
                leftY = () -> 0.0,
                leftX = () -> 0.9 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotation = () -> 0.0;

        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
        runScheduler(1);

        Assertions.assertEquals(1, swerveDrive.getPositions());
    }

//    @Test
    public void TestTeleopY() {
        DoubleSupplier
                leftY = () -> 0.9 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                leftX = () -> 0.0,
                rotation = () -> 0.0;

        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
        runScheduler(1);

        Assertions.assertEquals(10, swerveDrive.getPose().getY());

    }

//    @Test
    public void TestTeleopTheta() {
        DoubleSupplier
                leftY = () -> 0.0,
                leftX = () -> 0.0,
                rotation = () -> 0.9 * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
        runScheduler(1);

        Assertions.assertEquals(45, swerveDrive.getPose().getRotation().getDegrees());
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
