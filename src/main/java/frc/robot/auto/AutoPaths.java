// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.AutoConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.DefaultArmElevatorDriveConfig;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoChooser;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorHeight;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.HashMap;

public class AutoPaths {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private HashMap<String, Command> autoEventMap = new HashMap<>();

  public AutoPaths(
      SwerveDrive swerveSubsystem,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public void sendCommandsToChooser() {
    AutoChooser.createSinglePath("Do Nothing", new InstantCommand());

    if (swerveSubsystem == null)
      return;
    if (intakeSubsystem == null)
      return;
    if (armSubsystem == null)
      return;
    if (elevatorSubsystem == null)
      return;

    autoEventMap.put(
        "intakeCone",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
            new SetArmAngle(armSubsystem, ArmConstants.kArmGroundIntakeRotation),
            new IntakeCone(intakeSubsystem)));
    autoEventMap.put(
        "intakeCube",
        new ParallelCommandGroup(
            new SetElevatorHeight(elevatorSubsystem, Elevator.ElevatorPosition.GROUND_INTAKE),
            new SetArmAngle(armSubsystem, ArmConstants.kArmGroundIntakeRotation),
            new IntakeCube(intakeSubsystem)));
    autoEventMap.put(
        "defaultPosition",
        new ParallelCommandGroup(
            new DefaultArmElevatorDriveConfig(elevatorSubsystem, armSubsystem),
            new IntakeOff(intakeSubsystem)));

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem, autoEventMap);
    Command scorePreload = new IntakeCube(intakeSubsystem);

    ArrayList<Command> node2PreloadEngage = autoBuilder.createPaths("Node2-Preload-Engage", kDefaultPathConstraints);
    AutoChooser.addIncrementalPaths(scorePreload, "Node2-Preload-Engage", node2PreloadEngage);

    AutoChooser.sendChooserToDashboard("Auto Chooser");
  }

  public Command getSelectedPath() {
    return AutoChooser.getCommand();
  }
}
