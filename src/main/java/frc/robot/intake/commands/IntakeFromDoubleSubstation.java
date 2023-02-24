// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.arm.ArmConstants.kArmAngleDoubleSubstation;
import static frc.robot.elevator.ElevatorConstants.kElevatorHeightDoubleSubstation;
import static frc.robot.intake.IntakeConstants.IntakeFromDoubleSubstation.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.led.LED;
import frc.robot.led.commands.LEDSetAllSectionsPattern;
import frc.robot.led.patterns.DoubleSubstationPattern;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.List;

public class IntakeFromDoubleSubstation extends CommandBase {
  private SwerveDrive swerveSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;
  private AutoBuilder autoBuilder;
  private LED ledSubsystem;

  public enum GamePiece {
    CONE,
    CUBE
  }

  public enum SubstationSide {
    RIGHT,
    LEFT
  }

  private GamePiece gamePiece;
  private SubstationSide substationSide;
  private DriverStation.Alliance alliance;

  public IntakeFromDoubleSubstation(
      SwerveDrive swerveSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      LED ledSubsystem,
      GamePiece gamePiece,
      SubstationSide substationSide) {

    this.swerveSubsystem = swerveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gamePiece = gamePiece;
    this.substationSide = substationSide;
    this.alliance = DriverStation.getAlliance();

    autoBuilder = new AutoBuilder(swerveSubsystem);

    addRequirements(
        swerveSubsystem, elevatorSubsystem, armSubsystem, ledSubsystem, intakeSubsystem);
  }

  // TODO: check the set arm angle after merged with the fixed arm commands
  @Override
  public void initialize() {

    List<PathPoint> waypoints = new ArrayList<>();

    // Right Double Substation for Blue Alliance
    waypoints.add(
        new PathPoint(
            swerveSubsystem.getPose().getTranslation(), swerveSubsystem.getPose().getRotation()));

    if (alliance == DriverStation.Alliance.Blue) {
      if (substationSide == SubstationSide.LEFT) {
        waypoints.add(kLeftSubstationBlue);
      } else if (substationSide == SubstationSide.RIGHT) waypoints.add(kRightSubstationBlue);
    } else {
      if (substationSide == SubstationSide.LEFT) {
        waypoints.add(kLeftSubstationRed);
      } else if (substationSide == SubstationSide.RIGHT) waypoints.add(kRightSubstationRed);
    }

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(kPathContraints, waypoints);

    DoubleSubstationPattern doubleSubstationPattern = new DoubleSubstationPattern(gamePiece);

    Command intakeGamePieceCommand;

    switch (gamePiece) {
      case CONE:
        intakeGamePieceCommand = new IntakeCone(intakeSubsystem);
        break;
      case CUBE:
        intakeGamePieceCommand = new IntakeCube(intakeSubsystem);
        break;
      default:
        intakeGamePieceCommand = new IntakeCube(intakeSubsystem);
        break;
    }

    Command autoDoubleSubstation =
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, kElevatorHeightDoubleSubstation),
            new SetArmAngle(armSubsystem, kArmAngleDoubleSubstation),
            intakeGamePieceCommand,
            autoBuilder.createPathPlannerCommand(trajectory, false),
            new LEDSetAllSectionsPattern(ledSubsystem, doubleSubstationPattern));

    autoDoubleSubstation.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
