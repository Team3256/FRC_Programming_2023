package frc.robot.auto.helpers;


import edu.wpi.first.math.geometry.Rotation2d;

public class UniformThetaSupplier implements ThetaSupplier {
    private double trajectoryDuration;
    private Rotation2d desiredRotation;
    private Rotation2d initialAngle = new Rotation2d();
    private double proportion;

    public UniformThetaSupplier(double trajectoryDuration, Rotation2d desiredRotation, Rotation2d initialAngle, double proportion) {
        this.trajectoryDuration = trajectoryDuration;
        this.desiredRotation = desiredRotation;
        this.initialAngle = initialAngle;
        this.proportion = proportion;
    }

    public UniformThetaSupplier(double trajectoryDuration, Rotation2d desiredRotation, double proportion) {
        this.trajectoryDuration = trajectoryDuration;
        this.desiredRotation = desiredRotation;
        this.proportion = proportion;
    }

    public UniformThetaSupplier(double trajectoryDuration) {
        this.trajectoryDuration = trajectoryDuration;
        this.desiredRotation = new Rotation2d();
        this.proportion = 1;
    }

    public UniformThetaSupplier(Rotation2d desiredRotation, double proportion) {
        this.trajectoryDuration = 5; // SHOULD GET CHANGED BY TRAJECTORY FACTORY
        this.desiredRotation = desiredRotation;
        this.proportion = proportion;
    }

    public UniformThetaSupplier(Rotation2d desiredRotation, Rotation2d initialAngle, double proportion) {
        this.trajectoryDuration = 5; // SHOULD GET CHANGED BY TRAJECTORY FACTORY
        this.desiredRotation = desiredRotation;
        this.initialAngle = initialAngle;
        this.proportion = proportion;
    }

    public void setTrajectoryDuration(double duration) {
        this.trajectoryDuration = duration;
    }

    public Rotation2d rotationSupply(double now) {
        return new Rotation2d((this.desiredRotation.getRadians() >= 0 ? 1 : -1) *
                (this.initialAngle.getRadians()
                + Math.min(
                    Math.abs(
                            this.desiredRotation.getRadians() * (now/(this.trajectoryDuration * proportion))
                    ),
                    Math.abs(
                            this.desiredRotation.getRadians()
                    )
                )
        ));
    }
}

