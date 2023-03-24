package frc.robot.limelight.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.limelight.Limelight;
import frc.robot.swerve.SwerveDrive;

import static frc.robot.Constants.VisionConstants.FrontConstants.kLimelightNetworkTablesName;
import static frc.robot.Constants.VisionConstants.kNeuralDetectiorPipelineIndex;
public class AlignWithGamePiece extends PIDCommand {

  private final SwerveDrive swerveDrive;

  /**
   * Creates a new AutoDetectGamePieces.
   */
  public AlignWithGamePiece(SwerveDrive swerveDrive) {
    super();

    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setPipelineIndex(kLimelightNetworkTablesName, kNeuralDetectiorPipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
