package frc.robot.flywheel.commands;

import static org.junit.Assert.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeForward;
import frc.robot.intake.commands.Outtake;

import org.junit.*;
import org.junit.Before;

public class SetFlywheelFromPIDTest {
    public static final double DELTA = 20; // acceptable deviation range
    public static final double PID_TIMEOUT = 2;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        CommandScheduler.getInstance().enable();
        DriverStationSim.setEnabled(true);

    }

    private void testOvershoot(double rpm) {
        Intake flywheelSubsystem = new Intake();

        SetFlywheelFromPID pidCommand = new SetFlywheelFromPID(flywheelSubsystem, rpm);
        BooleanSupplier overshootTest = () -> flywheelSubsystem.getAngularVelocityRPM() <= rpm + DELTA;

        CommandScheduler.getInstance().schedule(pidCommand);
        runSchedulerWithAssert(0.75, overshootTest);
    }

    private void testPIDResponse(double rpm, double timeout) {
        FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
        SetFlywheelFromPID pidCommand = new SetFlywheelFromPID(flywheelSubsystem, rpm);
        BooleanSupplier endCondition = () -> Math.abs(flywheelSubsystem.getAngularVelocityRPM() - rpm) < DELTA;

        CommandScheduler.getInstance().schedule(pidCommand);
        double timeUntilSetpoint = runSchedulerUntil(endCondition, PID_TIMEOUT);

        if (timeUntilSetpoint != -1 && timeUntilSetpoint <= timeout) {
            System.out.println("Your PIDController took " + timeUntilSetpoint + " seconds to reach the setpoint of " + rpm + ".");
        } else {
            System.out.println("Your PIDController took " + timeUntilSetpoint + " seconds (-1 means it never reached the setpoint) to reach the setpoint of " + rpm + ", but should have taken less than " + timeout + " seconds.");
        }
        assertFalse(timeUntilSetpoint == -1); // did not reach setpoint at all
        assertTrue(timeUntilSetpoint <= timeout); // made it to setpoint
    }

    @Test
    public void testOvershootWith1200RPM() {
        testOvershoot(1200);
    }

    @Test
    public void testOvershootWith1800RPM() {
        testOvershoot(1800);
    }

    @Test
    public void testOvershootWith2200RPM() {
        testOvershoot(2200);
    }

    @Test
    public void testOvershootWith2500RPM() {
        testOvershoot(2500);
    }

    @Test
    public void testOvershootWith3000RPM() {
        testOvershoot(3000);
    }

    @Test
    public void testResponseWith1200RPM() {
        testPIDResponse(1200, 0.11);
    }

    @Test
    public void testResponseWith1800RPM() {
        testPIDResponse(1800, 0.11);
    }

    @Test
    public void testResponseWith2200RPM() {
        testPIDResponse(2200, 0.13);
    }

    @Test
    public void testResponseWith2500RPM() {
        testPIDResponse(2500, 0.13);
    }

    @Test
    public void testResponseWith3000RPM() {
        testPIDResponse(3000, 0.17);
    }

    private static void runSchedulerWithAssert(double seconds, BooleanSupplier test) {
        try {
            for (int i = 0; i < seconds * 1000 / 20; ++i) {
                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
                CommandScheduler.getInstance().run();

                Thread.sleep(20); // run every 20ms
                assertEquals(test.getAsBoolean(), true);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static double runSchedulerUntil(BooleanSupplier endCondition, double timeout) {
        try {
            for (int i = 0; i < timeout * 1000 / 20; ++i) {
                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
                CommandScheduler.getInstance().run();
                Thread.sleep(20);
                if (endCondition.getAsBoolean()) return i * 0.02;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return -1;
    }

    private static void runScheduler(double seconds) {
        try {
            for (int i = 0; i < seconds * 1000 / 20; ++i) {
                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
                CommandScheduler.getInstance().run();
                Thread.sleep(20);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
