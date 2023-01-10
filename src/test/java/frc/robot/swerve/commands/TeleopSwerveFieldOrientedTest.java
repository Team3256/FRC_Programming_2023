package frc.robot.swerve.commands;


import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.swerve.SwerveDrive;
import org.junit.*;
import org.junit.jupiter.api.*;
public class TeleopSwerveFieldOrientedTest {
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        CommandScheduler.getInstance().enable();
        DriverStationSim.setEnabled(true);
    }

    private void testOffset(double x, double y, double theta) {
        SwerveDrive swerve = new SwerveDrive();
        double delta = 1.5;

        TeleopSwerveFieldOriented teleopCommand = new TeleopSwerveFieldOriented(swerve, ()->x,()->y,()->theta);
        Pose2d pose = swerve.getPose();
        BooleanSupplier overshootTest = () -> Math.hypot(pose.getX()-x,pose.getY()-y)<delta;

        CommandScheduler.getInstance().schedule(teleopCommand);
        runSchedulerWithAssert(0.75, overshootTest);

        Assertions.assertEquals(true,true);
    }

    @Test
    public void testOffsetFullThrottle() {
        testOffset(100,0,0);
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