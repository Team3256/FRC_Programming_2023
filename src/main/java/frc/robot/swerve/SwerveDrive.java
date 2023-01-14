package frc.robot.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.helpers.SwerveModule;

import static frc.robot.Constants.SwerveConstants.*;


public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(0, Mod0.constants);
    private final SwerveModule frontRightModule = new SwerveModule(1, Mod1.constants);
    private final SwerveModule backLeftModule = new SwerveModule(2, Mod2.constants);
    private final SwerveModule backRightModule = new SwerveModule(3, Mod3.constants);

    private final double[] velocities;
    private ChassisSpeeds lastChassisSpeeds;
    public boolean isSpeedSet;

    private final SwerveModule[] swerveModules = {
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule
    };

    private final SwerveDriveOdometry odometry;
    private final PigeonIMU gyro;

    public SwerveDrive() {
        gyro = new PigeonIMU(pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        odometry = new SwerveDriveOdometry(
            swerveKinematics,
            getYaw(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        velocities = new double[4];
        isSpeedSet = false;
        lastChassisSpeeds = new ChassisSpeeds();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    private void setModuleVelocity(SwerveModuleState[] swerveModuleStates) {
        for(int mod=0;mod<4;mod++){
            velocities[mod] = swerveModuleStates[mod].speedMetersPerSecond;
        }
    }

    public double[] getModuleVelocity() {
        return this.velocities;
    }

    public void setLastChassisSpeeds(ChassisSpeeds chassisSpeed) {
        if (!this.isSpeedSet) {
            this.lastChassisSpeeds = chassisSpeed;
            this.isSpeedSet = true;
        }
    }

    public ChassisSpeeds getLastChassisSpeed() {
        return this.lastChassisSpeeds;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds currChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
        );

        setLastChassisSpeeds(currChassisSpeed);

        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        setModuleVelocity(swerveModuleStates);

        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            states[mod.moduleNumber] = mod.getPosition();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        odometry.update(getYaw(), getPositions());

        for(SwerveModule mod : swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Distance", mod.getPosition().distanceMeters);
        }
    }
}
