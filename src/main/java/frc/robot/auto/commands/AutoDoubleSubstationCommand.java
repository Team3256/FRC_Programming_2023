// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.ArmSubsystem;
import frc.robot.elevator.Elevator;
import frc.robot.swerve.SwerveDrive;
import frc.robot.arm.ArmSubsystem;
import frc.robot.elevator.Elevator;

public class AutoDoubleSubstationCommand extends CommandBase {
    private SwerveDrive swerveSubsystem;
    private Elevator elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    public AutoDoubleSubstationCommand(SwerveDrive swerveSubsystem, Elevator elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        addRequirements(swerveSubsystem, elevatorSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }
}