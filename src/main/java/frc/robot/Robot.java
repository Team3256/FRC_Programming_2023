// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.arm.Arm.loadArmPreferences;
import static frc.robot.elevator.Elevator.loadElevatorPreferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Constants.RobotMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    logger.recordMetadata("ProjectName", "WarriorBorgs (2023)"); // Set a metadata value
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    Constants.RobotMode currentMode = Constants.kCurrentMode;
    if (isReal()) {
      currentMode = RobotMode.REAL;
      System.out.println("Robot is real, forcing robot mode to REAL");
    }

    switch (currentMode) {
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
      default: // Unknown mode.
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;
    }

    logger.start(); // Start advkit logger

    if (FeatureFlags.kUsePrefs) {
      loadArmPreferences();
      loadElevatorPreferences();
    }
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Run tests
    if (FeatureFlags.kCanTestEnabled) robotContainer.CANTest();

    if (FeatureFlags.kPitRoutineEnabled) robotContainer.startPitRoutine();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
