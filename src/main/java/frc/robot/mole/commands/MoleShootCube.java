package frc.robot.mole.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.mole.Mole;

public class MoleShootCube extends CommandBase {

    Mole moleSubsytem;
    Rotation2d desiredMoleAngle;

    public MoleShootCube(Mole moleSubsystem, Rotation2d desiredMoleAngle){
        this.moleSubsytem = moleSubsystem;
        this.desiredMoleAngle = desiredMoleAngle;
        addRequirements(moleSubsystem);
    }

    @Override
    public void initialize() {
        moleSubsytem.setPivotPosition(desiredMoleAngle.getDegrees());
    }

    @Override
    public void execute() {
        moleSubsytem.outtakeCube();
    }

    @Override
    public void end(boolean interrupted) {
        moleSubsytem.off();
    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
