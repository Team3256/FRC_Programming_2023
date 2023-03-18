package frc.robot.mole;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.ArmConstants;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.logging.Loggable;
import frc.robot.mole.commands.MoleIntakeCube;
import frc.robot.swerve.helpers.Conversions;

import static frc.robot.Constants.ShuffleboardConstants.*;
import static frc.robot.arm.ArmConstants.kArmGearing;
import static frc.robot.mole.MoleConstants.*;

public class Mole extends SubsystemBase implements Loggable, CANTestable {
    public enum MolePosition {
        CUBE_LOW(MoleConstants.kDefaultMoleAngle),
        CUBE_MID(MoleConstants.kCubeMidRotation),
        CUBE_HIGH(MoleConstants.kCubeHighRotation);

        public final Rotation2d rotation;

        private MolePosition(Rotation2d rotation) {
            this.rotation = rotation;
        }
    }
    private WPI_TalonFX moleScoreMotor;
    private WPI_TalonFX molePivotMotor;

    public Mole(){
        if(RobotBase.isReal()){
            configureRealHardware();
        } else {
            configureSimHardware();
        }

        off();
        System.out.println("Mole Intake Initialized");
    }

    private void configureRealHardware(){
        moleScoreMotor = TalonFXFactory.createDefaultTalon(kMoleCANDevice);
        moleScoreMotor.setNeutralMode(NeutralMode.Brake);

        molePivotMotor = TalonFXFactory.createDefaultTalon(kMolePivotCANDevice);
        molePivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void configureSimHardware(){
        moleScoreMotor = new WPI_TalonFX(kMoleMotorID);
        moleScoreMotor.setNeutralMode(NeutralMode.Brake);

        molePivotMotor = new WPI_TalonFX(kMolePivotMotorID);
        molePivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getMoleSpeed(){
        return moleScoreMotor.getMotorOutputPercent();
    }

    public double getMolePivotPositionRadians(){
        return Conversions.falconToRadians(molePivotMotor.getSelectedSensorPosition(), kMolePivotGearing);
    }

    public void keepCube(){
        moleScoreMotor.set(ControlMode.Current, -kMoleKeepingCurrent);
    }

    public void intakeCube(){
        System.out.println("Intake Cube");
        moleScoreMotor.set(ControlMode.PercentOutput, kMoleCubeSpeed);
    }

    public void setPivotPosition(double desiredAngle) {
        molePivotMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredAngle, kArmGearing));
    }

    public void outtakeCube() {
        moleScoreMotor.set(ControlMode.Velocity, 4);
    }


    public boolean isCurrentSpiking(){
        return moleScoreMotor.getSupplyCurrent() >= kMoleCurrentSpikingThreshold;
    }

    public void off(){
       System.out.println("Mole Off");
       moleScoreMotor.neutralOutput();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Mole Current", moleScoreMotor.getSupplyCurrent());
    }

    public void logInit(){
        getLayout(kDriverTabName).add(this);
        getLayout(kDriverTabName).add(new MoleIntakeCube(this));
        getLayout(kDriverTabName).add(moleScoreMotor);
    }

    public ShuffleboardLayout getLayout(String tab){
        return Shuffleboard.getTab(tab)
                .getLayout(kMoleLayoutName, BuiltInLayouts.kList)
                .withSize(2,4);
    }

    public boolean CANTest(){
        System.out.println("Testing Mole CAN:");
        boolean result = CANDeviceTester.testTalonFX(moleScoreMotor);
        System.out.println("Mole CAN connected" + result);
        SmartDashboard.putBoolean("Mole CAN connected", result);
        return result;
    }
}
