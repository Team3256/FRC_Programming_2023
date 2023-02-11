// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.UnitTestBase;
import frc.robot.swerve.SwerveConstants;
import frc.robot.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.*;

public class TeleopSwerveTests extends UnitTestBase {
  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0);
    CommandScheduler.getInstance().enable();
    DriverStationSim.setEnabled(true);
  }

  @Test
  public void testSwerveForward() {
    SwerveDrive swerveSubsystem = new SwerveDrive();
    DoubleSupplier translationSupplier = () -> 0.5;
    DoubleSupplier strafeSupplier = () -> 0.5;
    DoubleSupplier rotationSupplier = () -> 0.5;
    TeleopSwerve teleopSwerve =
        new TeleopSwerve(
            swerveSubsystem, translationSupplier, strafeSupplier, rotationSupplier, true, true);

    swerveSubsystem.frontLeftModule.setDesiredState(
        new SwerveModuleState(100.0, Rotation2d.fromDegrees(45)), true);
    swerveSubsystem.frontLeftModule.setDesiredState(
        new SwerveModuleState(100.0, Rotation2d.fromDegrees(45)), true);
    swerveSubsystem.frontLeftModule.setDesiredState(
        new SwerveModuleState(100.0, Rotation2d.fromDegrees(45)), true);
    swerveSubsystem.frontLeftModule.setDesiredState(
        new SwerveModuleState(100.0, Rotation2d.fromDegrees(45)), true);

    //    teleopSwerve.initialize();
    //    swerveSubsystem.simulationPeriodic();
    //    teleopSwerve.execute();
    //    swerveSubsystem.simulationPeriodic();
    //    this.runScheduler(1, teleopSwerve, swerveSubsystem);

    //    SwerveModulePosition[] states = swerveSubsystem.getModulePositions();

    //    System.out.println("Degrees are: " + states[0].angle.getDegrees());
    System.out.println("Default Degrees are: " + SwerveConstants.FrontLeft.kAngleOffset);
    System.out.println("Swerve pose: " + swerveSubsystem.getPose());

    //    System.out.println(
    //        "Distance in meters "
    //            + positions[0].distanceMeters
    //            + " and degrees "
    //            + positions[0].angle.getDegrees());
    //
    //    Assertions.assertAll(
    //        () -> {
    //          Assertions.assertEquals(0.0, positions[0].distanceMeters);
    //          Assertions.assertEquals(
    //              0.0,
    //              positions[0].angle.getDegrees() +
    // Constants.SwerveConstants.FrontLeft.angleOffset,
    //              1);
    //        });
  }

  //  private static void runScheduler(double seconds) {
  //    try {
  //      Thread.sleep(1000);
  //      System.out.println("About to run the scheduler");
  //      for (int i = 0; i < seconds * 1000 / 200; ++i) {
  //        System.out.println("A");
  //        com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(100);
  //        System.out.println("B");
  //        CommandScheduler.getInstance().run();
  //        System.out.println("C");
  //        Thread.sleep(200);
  //        System.out.println("D");
  //      }
  //    } catch (InterruptedException e) {
  //      e.printStackTrace();
  //    }
  //  }
}
