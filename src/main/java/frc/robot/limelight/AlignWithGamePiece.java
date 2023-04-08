// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.swerve.SwerveDrive;

import static frc.robot.Constants.VisionConstants.FrontConstants.kLimelightNetworkTablesName;
import static frc.robot.Constants.VisionConstants.*;

public class AlignWithGamePiece extends PIDCommand {

  private static SwerveDrive swerveDrive = new SwerveDrive();
  private static int pipelineIndex;
  private static int previousPipelineIndex;

  /** Creates a new AlignWithGamePiece. */
  public AlignWithGamePiece(SwerveDrive swerveDrive, int pipelineIndex) {
    super(
        new PIDController(kAutoAlignP, kAutoAlignI, kAutoAlignD),
        // Close loop on heading
        AlignWithGamePiece::getAngleToTarget,
        // Set reference to target
        0,
        // Pipe output to move robot
        output -> AlignWithGamePiece.move(output),
        // Require the drive
        swerveDrive);

    AlignWithGamePiece.pipelineIndex = pipelineIndex;
    AlignWithGamePiece.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command Initialized");
    System.out.println("**********************");

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

  public static void move(double angle) {
    System.out.println("**********************");
    System.out.println("Align with Game Piece Command::MoveALittle");
    System.out.println("**********************");

    // Find how fast to move
    double X = angle * kAngleSpeedMultiplier;
    // Move the robot
    swerveDrive.drive(new Translation2d(X, 0), 0, false, false);
  }
}
