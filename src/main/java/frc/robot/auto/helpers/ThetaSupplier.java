package frc.robot.auto.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ThetaSupplier {
    Rotation2d rotationSupply(double now);
    void setTrajectoryDuration(double duration);
}
