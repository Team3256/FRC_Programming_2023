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
  private final int pipelineIndex;
  private static double latestTranslationX;

  /** Creates a new AlignWithGamePiece. */
  public AlignWithGamePiece(SwerveDrive swerveDrive, int pipelineIndex) {
    super(
        new PIDController(kAutoAlignP, kAutoAlignI, kAutoAlignD),
        // Close loop on heading
        AlignWithGamePiece::getTranslationX,
        // Set reference to target
        0,
        // Pipe output to turn robot
        output -> AlignWithGamePiece.moveRobotX(output),
        // Require the drive
        swerveDrive);

    this.pipelineIndex = pipelineIndex;
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setPipelineIndex(kLimelightNetworkTablesName, pipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(latestTranslationX) < kLimeLightTranslationAutoAlignThreshold;
  }

  // return the tx value from the latest limelight targeting.
  public static double getTranslationX() {
    LimelightResults latestResults = Limelight.getLatestResults(kLimelightNetworkTablesName);
    double[] robotSpace = Limelight.getBotPose_TargetSpace(kLimelightNetworkTablesName);
    double distance = robotSpace[1];
    double angleToTarget = latestResults.targetingResults.targets_Detector[0].tx;

    latestTranslationX = Math.tan(angleToTarget) * distance;
    return latestTranslationX;
  }

  public static void moveRobotX(double tx) {
    swerveDrive.drive(new Translation2d(tx, 0), 0, true, true);
  }
}
