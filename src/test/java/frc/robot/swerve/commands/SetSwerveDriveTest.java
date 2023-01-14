//package frc.robot.swerve.commands;
//
//import edu.wpi.first.hal.HAL;
//import edu.wpi.first.wpilibj.simulation.DriverStationSim;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Constants;
//import frc.robot.swerve.SwerveDrive;
//import frc.robot.swerve.SwerveModuleIO;
//import org.junit.jupiter.api.Assertions;
//import org.junit.jupiter.api.BeforeAll;
//import org.junit.jupiter.api.Test;
//
//import java.util.Arrays;
//import java.util.function.DoubleSupplier;
//
//public class SetSwerveDriveTest {
//
//    public static final double DELTA = 0.5; // acceptable deviation range
//
//    @BeforeAll
//    public static void setup() {
//        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
//        CommandScheduler.getInstance().enable();
//        DriverStationSim.setEnabled(true);
//    }
//    //    @Test
//    public void dummyTest(){
//        Assertions.assertTrue(true);
//    }
//    //    @Test
//    public void TestVelocities() {
//        SwerveDrive swerveDrive = new SwerveDrive(new SwerveModuleIO());
//        DoubleSupplier
//                leftY = () -> 0.1 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                leftX = () -> 0.1 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                rotation = () -> 0.2;
//
//        TeleopSwerveFieldOriented command = new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation);
//
//        runScheduler(3, command);
//
//        System.out.println("Was drive method run by command? " + swerveDrive.isSpeedSet);
//        System.out.println("Velocities that are currently set: " + Arrays.toString(swerveDrive.getModuleVelocity()));
//        System.out.println("First Chassis Speed set: " + swerveDrive.getLastChassisSpeed());
//
////        Assertions.assertEquals(ChassisSpeeds.fromFieldRelativeSpeeds(4.47, 2.48, 0, new Rotation2d()), swerveDrive.getLastChassisSpeed());
//    }
//
//    //    @Test
//    public void TestTeleopX() {
//
//        SwerveDrive swerveDrive = new SwerveDrive(new SwerveModuleIO());
//
//        DoubleSupplier
//                leftY = () -> 0.0,
//                leftX = () -> 0.9 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                rotation = () -> 0.0;
//
//        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
////        runScheduler(1);
//
//        Assertions.assertEquals(1, swerveDrive.getPositions());
//    }
//
//    //    @Test
//    public void TestTeleopY() {
//        SwerveDrive swerveDrive = new SwerveDrive(new SwerveModuleIO());
//
//        DoubleSupplier
//                leftY = () -> 0.9 * Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
//                leftX = () -> 0.0,
//                rotation = () -> 0.0;
//
//        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
////        runScheduler(1);
//
//        Assertions.assertEquals(10, swerveDrive.getPose().getY());
//
//    }
//
//    //    @Test
//    public void TestTeleopTheta() {
//        SwerveDrive swerveDrive = new SwerveDrive(new SwerveModuleIO());
//
//        DoubleSupplier
//                leftY = () -> 0.0,
//                leftX = () -> 0.0,
//                rotation = () -> 0.9 * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
//
//        CommandScheduler.getInstance().schedule(new TeleopSwerveFieldOriented(swerveDrive, leftX, leftY, rotation));
////        runScheduler(1);
//
//        Assertions.assertEquals(45, swerveDrive.getPose().getRotation().getDegrees());
//    }
//
//    private static void runScheduler(double seconds, Command command) {
//
//        for(int i=0; i<10; i++) {
//            try {
//                com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
//                CommandScheduler.getInstance().run();
//                command.execute();
//                Thread.sleep(20);
//
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//}
