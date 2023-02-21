package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;

import static frc.robot.climb.ClimbConstants.*;

public class Climb extends SubsystemBase implements CANTestable {

    private WPI_TalonFX climbMotor;

    public Climb(){
        if (RobotBase.isReal()){
            configureRealHardware();
        } else {
            configureSimHardware();
        }

        off();
        System.out.println("Climber Init");
    }

    public void configureRealHardware(){
        climbMotor = TalonFXFactory.createDefaultTalon(kClimbCANDevice);
        climbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void configureSimHardware(){
        climbMotor = new WPI_TalonFX(kClimbMotorID);
        climbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void deployClimb(){
        System.out.println("Climb Down");
        climbMotor.set(ControlMode.Position, kClimbDeployPosition);
    }

    public void retractClimb(){
        System.out.println("Climb Up");
        climbMotor.set(ControlMode.Position, kClimbRetractPosition);
    }

    public void off(){
        System.out.println("Climb Off");
        climbMotor.neutralOutput();
    }

    public boolean CANTest(){
        System.out.println("Testing Climb CAN:");
        boolean result = CANDeviceTester.testTalonFX(climbMotor);
        System.out.println("Climb CAN connected: " + result);
        SmartDashboard.putBoolean("Climb CAN connected", result);
        return result;
    }
}