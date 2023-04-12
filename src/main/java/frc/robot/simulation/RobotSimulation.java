// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import static frc.robot.Constants.*;
import static frc.robot.elevator.ElevatorConstants.kElevatorAngleOffset;
import static frc.robot.intake.IntakeConstants.kIntakeWristRatio;
import static frc.robot.simulation.SimulationConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.swerve.SwerveDrive;

public class RobotSimulation {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Arm armSubsystem;
  private Elevator elevatorSubsystem;
  private Mechanism2d robotCanvas;

  public RobotSimulation(
      SwerveDrive swerveSubsystem,
      Intake intakeSubsystem,
      Arm armSubsystem,
      Elevator elevatorSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
  }

  public void initializeRobot() {
    robotCanvas = new Mechanism2d(kRobotSimWindowWidth, kRobotSimWindowHeight);
    SmartDashboard.putData("Robot Sim", robotCanvas);

    MechanismRoot2d robotRoot = robotCanvas.getRoot("Robot Root", kRootX, kRootY);
    MechanismRoot2d elevatorRoot =
        robotCanvas.getRoot("Elevator Root", kElevatorRootX, kElevatorRootY);

    robotRoot.append(
        new MechanismLigament2d("Drive Chassis", kRobotLength, 0, 20, new Color8Bit(235, 137, 52)));
    MechanismLigament2d armPivot =
        new MechanismLigament2d(
            "Arm Pivot",
            Units.inchesToMeters(kArmPivotHeight),
            90,
            kArmLineWidth,
            new Color8Bit(Color.kBlue));

    if (kElevatorEnabled) {
      elevatorRoot.append(
          new MechanismLigament2d(
              "Initial Elevator Height",
              kMinElevatorExtension,
              Units.radiansToDegrees(kElevatorAngleOffset),
              kElevatorLineWidth,
              new Color8Bit(Color.kRed)));
      elevatorRoot.append(elevatorSubsystem.getLigament());

      elevatorSubsystem
          .getLigament()
          .append(
              new MechanismLigament2d(
                  "Elevator Right",
                  Units.inchesToMeters(6),
                  0,
                  kElevatorLineWidth,
                  new Color8Bit(Color.kRed)));
      elevatorSubsystem.getLigament().append(armPivot);
    }

    if (kArmEnabled) {
      MechanismLigament2d intakePivot = intakeSubsystem.getWrist();

      armPivot.append(armSubsystem.getLigament());
      armSubsystem.getLigament().append(intakePivot);

      if (kIntakeEnabled) {
        intakePivot
            .append(
                new MechanismLigament2d(
                    "Intake 1",
                    Units.inchesToMeters(3),
                    -3.728 + 90,
                    kIntakeLineWidth,
                    new Color8Bit(Color.kYellow)))
            .append(
                new MechanismLigament2d(
                    "Intake 2",
                    Units.inchesToMeters(6.813),
                    128.732,
                    kIntakeLineWidth,
                    new Color8Bit(Color.kYellow)))
            .append(
                new MechanismLigament2d(
                    "Intake 3",
                    Units.inchesToMeters(11.738),
                    92.834,
                    kIntakeLineWidth,
                    new Color8Bit(Color.kYellow)))
            .append(
                new MechanismLigament2d(
                    "Intake 4",
                    Units.inchesToMeters(10.5),
                    152.3,
                    kIntakeLineWidth,
                    new Color8Bit(Color.kYellow)));
      }
    }
  }

  public void updateSubsystemPositions() {
    if (kElevatorEnabled) {
      elevatorSubsystem
          .getLigament()
          .setLength(
              Units.inchesToMeters(kArmStartPosition) + elevatorSubsystem.getElevatorPosition());
    }
    if (kArmEnabled) {
      armSubsystem
          .getLigament()
          .setAngle(Units.radiansToDegrees(armSubsystem.getArmPositionElevatorRelative()) - 90);
    }
    if (kIntakeEnabled) {
      intakeSubsystem
          .getWrist()
          .setAngle(
              Units.radiansToDegrees(armSubsystem.getArmPositionGroundRelative())
                      * kIntakeWristRatio
                  - 90);
    }
  }

  public void addDoubleSubstation(GamePiece gamePiece) {
    MechanismRoot2d goalRoot = robotCanvas.getRoot("Goal Root", kGoalStationX, kRootY);
    MechanismRoot2d gamePieceRoot =
        robotCanvas.getRoot(
            "Game Piece Root", kGoalStationX, FieldConstants.LoadingZone.kDoubleSubstationShelfZ);

    goalRoot.append(
        new MechanismLigament2d(
            "Double Substation",
            FieldConstants.LoadingZone.kDoubleSubstationShelfZ,
            90,
            20,
            new Color8Bit((Color.kGreen))));

    if (gamePiece == GamePiece.CONE) {
      gamePieceRoot.append(
          new MechanismLigament2d(
              "Cone Tip", kConeTipHeight, 90, kConeTipLineWidth, new Color8Bit((Color.kYellow))));
      gamePieceRoot.append(
          new MechanismLigament2d(
              "Cone Base",
              kConeBaseHeight,
              90,
              kConeBaseLineWidth,
              new Color8Bit((Color.kYellow))));
    } else {
      gamePieceRoot.append(
          new MechanismLigament2d(
              "Cube", kCubeBase, 90, kCubeLineWidth, new Color8Bit((Color.kPurple))));
    }
  }
}
