package frc.robot.auto.helpers;

import frc.robot.swerve.SwerveDrive;


public class Paths {
    private static TrajectoryFactory trajectoryFactory;


    public static void initialize(SwerveDrive drive) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;

    }
}








