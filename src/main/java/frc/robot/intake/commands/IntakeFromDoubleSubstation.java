// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.IntakeFromDoubleSubstation.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private enum GamePiece {
    CONE,
    CUBE
  }

  private GamePiece gamePiece;

  public IntakeFromDoubleSubstation(
      SwerveDrive swerveSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      Intake intakeSubsystem,
      LED ledSubsystem,
      GamePiece gamePiece) {
    this.swerveSubsystem = swerveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.gamePiece = gamePiece;
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
    waypoints.add(new PathPoint(new Translation2d(14.25, 6.03), new Rotation2d(180)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(new PathConstraints(5, 5), waypoints);

    DoubleSubstationPattern doubleSubstationPattern = new DoubleSubstationPattern();

    Command intakeGamePieceCommand = null;

    switch (gamePiece) {
      case CONE:
        intakeGamePieceCommand = new IntakeCone(intakeSubsystem);
        break;
      case CUBE:
        intakeGamePieceCommand = new IntakeCube(intakeSubsystem);
        break;
    }

    Command autoDoubleSubstation =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetElevatorHeight(elevatorSubsystem, kElevatorHeight),
                new SetArmAngle(armSubsystem, new Rotation2d(kArmAngle))),
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
