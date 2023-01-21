package frc.robot.swerve.commands;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.PIDConstants.*;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;


// TODO: Use our own teleop command
public class TeleopSwerveWithAzimuth extends CommandBase {

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveDrive;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private ProfiledPIDController azimuthController;
  private int rotationXAxis;
  private int rotationYAxis;

  /** Driver control */
  public TeleopSwerveWithAzimuth(
      SwerveDrive swerveDrive,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationXAxis,
      int rotationYAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.azimuthController =
        new ProfiledPIDController(
            kAutoThetaControllerP,
            kAutoThetaControllerI,
            kAutoThetaControllerD,
            kAutoThetaControllerConstraints);

    
    azimuthController.enableContinuousInput(-180, 180);
    
  }

  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    // Obtains axis values for x and y for translation command

    double rAxisX = -controller.getRawAxis(rotationXAxis);
    // Gets position of joystick input on the x axis of the total stick area
    double rAxisY = -controller.getRawAxis(rotationYAxis);
    // Gets position of joystick input on the y axis of the total stick area
    
    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    rAxisX = (Math.abs(rAxisX) < Constants.azimuthStickDeadband) ? 0 : rAxisX;
    rAxisY = (Math.abs(rAxisY) < Constants.azimuthStickDeadband) ? 0 : rAxisY;
    // Safety area, insures that joystick movement will not be tracled within a certain area, prevents unintentional drifting
    
    double azimuthAngle = (Math.atan2(rAxisY, rAxisX) - Math.PI/2) * 180/Math.PI;
    // Converts from coordinates to angle, setsjoystick forward input as 0, converts angle to degrees

    double rotationPIDOutput = azimuthController.calculate(swerveDrive.getYaw().getDegrees(), azimuthAngle);
    // PID controller takes current robot position (getYaw) and compares to the azimuth angle to calculate error

    translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
    swerveDrive.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
    // Sets motors to the velocities defined here
  }
}

