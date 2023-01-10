package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.LEDConstants.AUTO_AIM_PATTERN;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.*;



public class AutoAlignDriveCommand extends CommandBase {
    private RobotLogger logger = new RobotLogger(AutoAlignDriveCommand.class.getCanonicalName());

    PIDController autoAlignVisionPIDController;
    PIDController autoAlignOdometryPIDController;

    DoubleSupplier driverJoystickX;
    DoubleSupplier driverJoystickY;
    DoubleSupplier operatorJoystickX;
    BooleanSupplier isShooting = () -> true;

    SwerveDrive swerveDrive;

    /**
     * Continuously rotates swerve drive toward Limelight target.
     *
     * @param drivetrainSubsystem drivetrain instance
     * @param driverJoystickX Driver's Translation X
     * @param driverJoystickY Driver's Translation Y
     */

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem,
                                 DoubleSupplier driverJoystickX,
                                 DoubleSupplier driverJoystickY,
                                 DoubleSupplier operatorJoystickX,
                                 BooleanSupplier isShooting
    ) {

        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = driverJoystickX;
        this.driverJoystickY = driverJoystickY;
        this.operatorJoystickX = operatorJoystickX;

        this.isShooting = isShooting;

        addRequirements(drivetrainSubsystem);

    }

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem,
                                 DoubleSupplier driverJoystickX,
                                 DoubleSupplier driverJoystickY,
                                 DoubleSupplier operatorJoystickX) {

        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = driverJoystickX;
        this.driverJoystickY = driverJoystickY;
        this.operatorJoystickX = operatorJoystickX;

        addRequirements(drivetrainSubsystem);

    }

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem) {
        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = () -> 0;
        this.driverJoystickY = () -> 0;
        this.operatorJoystickX = () -> 0;

        addRequirements(drivetrainSubsystem);

    }

    //The angle between the hub and the robot
    public double angleBetweenHub(Pose2d robotPose) {
        Translation2d hubCenteredRobotPosition = FieldConstants.HUB_POSITION.minus(robotPose.getTranslation());
        return Math.atan2(hubCenteredRobotPosition.getY(), hubCenteredRobotPosition.getX());
    }

    //The setpoint angle for the robot to turn towards the hub
    public double setAligningAngle(Pose2d robotPose) {
        return (Math.toDegrees(angleBetweenHub(robotPose))) % 360;
    }

    public void alignWithVision(){
        autoAlignVisionPIDController.setSetpoint(0);
        autoAlignVisionPIDController.enableContinuousInput(-180, 180);
    }

    public void alignWithoutVision(){
        autoAlignOdometryPIDController.setSetpoint(setAligningAngle(swerveDrive.getPose()));
        autoAlignOdometryPIDController.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        double autoAlignPidOutput = 0;

        SmartDashboard.putBoolean("Limelight Detected", Limelight.isTargetDetected());
        if(Limelight.isTargetDetected()){
            alignWithVision();
            autoAlignPidOutput = autoAlignVisionPIDController.calculate(Limelight.getTx());
//            swerveDrive.limelightLocalization(Limelight.getTunedDistanceToTarget(), Limelight.getTx());
            SmartDashboard.putNumber("Limelight Diatance", Limelight.getRawDistanceToTarget());
            SmartDashboard.putNumber("Swerve Turret Setpoint", Limelight.getTx());
        }
        else {
           // alignWithoutVision();
           // autoAlignPidOutput = autoAlignOdometryPIDController.calculate(swerveDrive.getEstimatedThetaOffset());
           // SmartDashboard.putNumber("Swerve Turret Setpoint",swerveDrive.getEstimatedThetaOffset());
        }

        //Save some Computation from Sqrt
        double speedSquared = Math.pow(driverJoystickX.getAsDouble(), 2) + Math.pow(driverJoystickY.getAsDouble(),2);

        // Use translation from Driver, Rotation is from PID
        // Ternary Explanation: Since while moving we can easily rotate, we don't mess with the PID
        // but while NOT moving, the motors need more power in order to actually move, so
        // we add a Constant, (Using copysign to either add or subtract depending on sign)
        double autoAlignPIDRotationalOutput = speedSquared > Math.pow(0.1, 2) ?
                autoAlignPidOutput :
                autoAlignPidOutput + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, autoAlignPidOutput);

        SmartDashboard.putNumber("Swerve Turret Output", autoAlignPidOutput);

        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverJoystickX.getAsDouble(),
                driverJoystickY.getAsDouble(),
                autoAlignPIDRotationalOutput +
                        (SWERVE_TURRET_OPERATOR_INFLUENCE*operatorJoystickX.getAsDouble()),
                swerveDrive.getGyroscopeRotation()
        ));
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
//        Limelight.enable();
        AUTO_AIM_PATTERN.update(true);
        autoAlignVisionPIDController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);
        autoAlignOdometryPIDController = new PIDController(SWERVE_ODOMETRY_TURRET_KP, SWERVE_ODOMETRY_TURRET_KI, SWERVE_ODOMETRY_TURRET_KD);
        SmartDashboard.putData("SWERVE TURREt", autoAlignVisionPIDController);
        logger.info("Auto Align Enabled");
    }

    @Override
    public void end(boolean interrupted) {
//        Limelight.disable();
        AUTO_AIM_PATTERN.update(false);
    }
}
