package frc.robot.auto.pathgeneration.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.dynamicpathgeneration.DynamicPathConstants;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.ErrorBlinkingPattern;
import frc.robot.led.patterns.SuccessBlinkingPattern;
import frc.robot.swerve.SwerveDrive;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

public class AutoScore extends CommandBase {
  public enum GridScoreHeight {
    HIGH,
    MID,
    LOW
  }

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;
  private GridScoreHeight gridScoreHeight;
  private BooleanSupplier isCurrentPieceCone;

  public AutoScore(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      GridScoreHeight gridScoreHeight,
      BooleanSupplier isCurrentPieceCone) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gridScoreHeight = gridScoreHeight;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  @Override
  public void initialize() {
    Pose2d src = swerveSubsystem.getPose();
    Pose2d finalSink = new Pose2d();
    Pose2d preSink = new Pose2d();

    int locationId = (int) SmartDashboard.getNumber("guiColumn", -1);
    if (locationId == -1) {
      System.out.println("locationId was invalid");
      if (ledSubsystem != null) {
        new LEDSetAllSectionsPattern(ledSubsystem, new ErrorBlinkingPattern()).schedule();
        return;
      }
    }

    switch (gridScoreHeight) {
      case HIGH:
        finalSink = kHighBlueScoringPoses[locationId];
      case MID:
        finalSink = kMidBlueScoringPoses[locationId];
      case LOW:
        finalSink = kBottomBlueScoringPoses[locationId];
    }
    preSink = kHighBlueScoringPoses[locationId];
    System.out.println("Running: Go to grid (id: " + locationId + ") from " + swerveSubsystem.getPose());

    Pose2d sink = DynamicPathConstants.kBlueTopDoubleSubstationPose;

    if (DriverStation.getAlliance() == Alliance.Red) {
      sink = PathUtil.flip(sink);
      preSink = PathUtil.flip(preSink);
    }

    // commands that will be run sequentially
    // TODO FINISH rest of command
    Command moveToPreSink = PathGeneration.createDynamicPath(swerveSubsystem.getPose(), preSink, swerveSubsystem);
    Command moveArmElevatorToPreset = new ParallelCommandGroup(
        new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.DOUBLE_SUBSTATION),
        new ConditionalCommand(
            new SetArmAngle(armSubsystem, Arm.ArmPosition.DOUBLE_SUBSTATION_CONE),
            new SetArmAngle(armSubsystem, Arm.ArmPosition.DOUBLE_SUBSTATION_CUBE),
            isCurrentPieceCone));

    Command runIntake = new ConditionalCommand(
        new IntakeCone(intakeSubsystem), new IntakeCube(intakeSubsystem), isCurrentPieceCone);
    Command moveToSubstation = PathGeneration.createDynamicPath(preSink, sink, swerveSubsystem);
    Command stopIntake = new IntakeOff(intakeSubsystem);
    Command stowArmElevator = new StowArmElevator(elevatorSubsystem, armSubsystem);
    LEDSetAllSectionsPattern signalLED = new LEDSetAllSectionsPattern(ledSubsystem, new SuccessBlinkingPattern());
    Command moveAwayFromSubstation = PathGeneration.createDynamicPath(sink, preSink, swerveSubsystem);

    // return sequential of all above commands
    Command finalCommand = Commands.sequence(
        moveToPreSink,
        Commands.deadline(runIntake.withTimeout(8), moveArmElevatorToPreset, moveToSubstation),
        Commands.deadline(moveAwayFromSubstation, stowArmElevator, stopIntake, signalLED));

    finalCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
