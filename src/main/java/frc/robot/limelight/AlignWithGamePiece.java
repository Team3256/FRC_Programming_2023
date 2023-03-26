// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.limelight;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.VisionConstants.FrontConstants.kLimeLightTranslationAutoAlignThreshold;
import static frc.robot.Constants.VisionConstants.FrontConstants.kLimelightNetworkTablesName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.swerve.SwerveDrive;

public class AlignWithGamePiece extends PIDCommand {

  private static SwerveDrive swerveDrive = new SwerveDrive();
  private static int pipelineIndex;
  private static int previousPipelineIndex;
  private static double latestTranslationX = 2 * kLimeLightTranslationAutoAlignThreshold;

  /** Creates a new AlignWithGamePiece. */
  public AlignWithGamePiece(SwerveDrive swerveDrive, int pipelineIndex) {
    super(
        new PIDController(kAutoAlignP, kAutoAlignI, kAutoAlignD),
        // Close loop on heading
        AlignWithGamePiece::getAngleToTarget,
        // Set reference to target
        0,
        // Pipe output to turn robot
        output -> AlignWithGamePiece.slideToTheRight(output),
        // Require the drive
        swerveDrive);

    this.pipelineIndex = pipelineIndex;
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command Initialized");
    System.out.println("**********************");

    // Get the bot pose
    double[] botPose = Limelight.getBotpose(kLimelightNetworkTablesName);
    // Archive the current pipeline index
    previousPipelineIndex = (int) Limelight.getCurrentPipelineIndex(kLimelightNetworkTablesName);
    // Set the pipeline index
    Limelight.setPipelineIndex(kLimelightNetworkTablesName, pipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getAngleToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command Ended");
    System.out.println("**********************");
    // Clean up
    Limelight.setPipelineIndex(kLimelightNetworkTablesName, previousPipelineIndex);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Get the angle to the target
    LimelightResults latestResults = Limelight.getLatestResults(kLimelightNetworkTablesName);
    if (latestResults.targetingResults.targets_Detector.length == 0) {
      return false;
    }
    double angleToTarget = latestResults.targetingResults.targets_Detector[0].tx;
    if (Math.abs(angleToTarget) < kAngleAlignmentThreshold) {
      System.out.println("**********************");
      System.out.println("Align with Game Piece Command Finished");
      System.out.println("**********************");
      return true;
    }
    return false;
  }

  // // return the tx value from the latest limelight targeting.
  // public static double getTranslationX() {
  //   System.out.println("**********************");
  //   System.out.println("Align with Game Piece Command::GetTranslationX");
  //   System.out.println("**********************");
  //   LimelightResults latestResults = Limelight.getLatestResults(kLimelightNetworkTablesName);
  //   double[] robotSpace = Limelight.getBotpose_wpiBlue(kLimelightNetworkTablesName);
  //   System.out.println("Latest Results: " + latestResults);
  //   System.out.println("JSON Dump: " + Limelight.getJSONDump(kLimelightNetworkTablesName));
  //   SmartDashboard.putNumberArray("RobotSpace", robotSpace);
  //   if (robotSpace.length < 1) {
  //     System.out.println("**********************");
  //     System.out.println("NO TARGETS FOUND");
  //     System.out.println("**********************");
  //     return 0;
  //   }
  //   System.out.println("**********************");
  //   System.out.println("TARGETS FOUND: " + robotSpace.length);
  //   System.out.println("**********************");
  //   double distance = robotSpace[1];
  //   double angleToTarget = latestResults.targetingResults.targets_Detector[0].tx;

  //   latestTranslationX = Math.tan(angleToTarget) * distance;
  //   SmartDashboard.putNumber("Latest Translation X", latestTranslationX);
  //   System.out.println("Latest Translation X");
  //   System.out.println(latestTranslationX);
  //   return 0;
  // }

  public static double getAngleToTarget() {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command::GetAngleToTarget");
    System.out.println("**********************");

    // Get the angle to the target
    LimelightResults latestResults = Limelight.getLatestResults(kLimelightNetworkTablesName);
    if (latestResults.targetingResults.targets_Detector.length == 0) {
      return 0;
    }
    double angleToTarget = latestResults.targetingResults.targets_Detector[0].tx;

    System.out.println("Latest Results: " + latestResults);
    System.out.println("JSON Dump: " + Limelight.getJSONDump(kLimelightNetworkTablesName));

    return angleToTarget;
  }

  public static void slideToTheRight(double angle) {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command::MoveALittle");
    System.out.println("**********************");

    // Find how much to move
    double X = angle * kAngleSpeedMultiplier;
    // Move the robot
    swerveDrive.drive(new Translation2d(X, 0), 0, false, false);
  }
}
