package frc.robot.swerve.commands;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.swerve.SwerveDrive;
import org.junit.jupiter.api.Test;

public class TeleopSwerveRobotOrientedTest {

    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        CommandScheduler.getInstance().enable();
        DriverStationSim.setEnabled(true);
    }

    private void testOffset(double x, double y, double theta) {
        SwerveDrive swerve = new SwerveDrive();
        double delta = 1.5;

        TeleopSwerveRobotOriented teleopCommand = new TeleopSwerveRobotOriented(swerve, ()->x,()->y,()->theta);
        Pose2d pose = swerve.getPose();
        BooleanSupplier overshootTest = () -> Math.hypot(pose.getX()-x,pose.getY()-y)<delta;

        CommandScheduler.getInstance().schedule(teleopCommand);
        runSchedulerWithAssert(0.75, overshootTest);
    }

    private void testResponseTime(double rpm, double timeout) {
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
        testPIDResponse(1200, 0.3);
    }

    @Test
    public void testResponseWith1800RPM() {
        testPIDResponse(1800, 0.35);
    }

    @Test
    public void testResponseWith2200RPM() {
        testPIDResponse(2200, 0.43);
    }

    @Test
    public void testResponseWith2500RPM() {
        testPIDResponse(2500, 0.49);
    }

    @Test
    public void testResponseWith3000RPM() {
        testPIDResponse(3000, 0.64);
    }

    private static void runSchedulerWithAssert(double seconds, BooleanSupplier test) {
        try {
            for (int i = 0; i < seconds * 1000 / 20; ++i) {
                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
                CommandScheduler.getInstance().run();
                Thread.sleep(20); // run every 20ms
                assert(test.getAsBoolean());
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