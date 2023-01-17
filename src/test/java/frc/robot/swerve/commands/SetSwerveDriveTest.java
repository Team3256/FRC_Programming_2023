package frc.robot.swerve.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class SetSwerveDriveTest {

  public static final double DELTA = 0.05; // acceptable deviation range

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
  }

  @Test
  public void TestTeleopY() {
    SwerveDrive swerveDrive = new SwerveDrive();
    DoubleSupplier leftY = () -> 0.1 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        leftX = () -> 0.0 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        rotation = () -> 0.0;

    TeleopSwerveFieldOriented command =
        new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation);

    runScheduler(3, command);

    System.out.println("Was drive method run by command? " + swerveDrive.isSpeedSet);
    System.out.println(
        "Velocities that are currently set: " + Arrays.toString(swerveDrive.getModuleVelocity()));
    System.out.println("Curr Chassis Speed set: " + swerveDrive.getLastChassisSpeed());

    Assertions.assertEquals(0.50, swerveDrive.getLastChassisSpeed().vyMetersPerSecond, DELTA);
  }

  @Test
  public void TestTeleopX() {

    SwerveDrive swerveDrive = new SwerveDrive();

    DoubleSupplier leftY = () -> 0.0,
        leftX = () -> 0.1 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        rotation = () -> 0.0;

    TeleopSwerveFieldOriented command =
        new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation);

    runScheduler(3, command);

    System.out.println("Was drive method run by command? " + swerveDrive.isSpeedSet);
    System.out.println(
        "Velocities that are currently set: " + Arrays.toString(swerveDrive.getModuleVelocity()));
    System.out.println("Curr Chassis Speed set: " + swerveDrive.getLastChassisSpeed());

    Assertions.assertEquals(0.5, swerveDrive.getLastChassisSpeed().vxMetersPerSecond, DELTA);
  }

  public void TestTeleopTheta() {
    SwerveDrive swerveDrive = new SwerveDrive();

    DoubleSupplier leftY = () -> 0.0,
        leftX = () -> 0.0,
        rotation = () -> 0.1 * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    //        Assertions.assertEquals(45, swerveDrive.getPose().getRotation().getDegrees());
  }

  private static void runScheduler(double seconds, Command command) {
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 10; i++) {
      try {
        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
        CommandScheduler.getInstance().run();
        command.execute();
        Thread.sleep(20);

      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
