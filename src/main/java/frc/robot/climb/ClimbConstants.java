package frc.robot.climb;

import frc.robot.drivers.CanDeviceId;

public final class ClimbConstants {
    //TODO Check CAN ID
    public static final int kClimbMotorID = 30;
    public static final String kClimbCANBus = "mani";

    public static final CanDeviceId kClimbCANDevice =  new CanDeviceId(kClimbMotorID, kClimbCANBus);

    public static final double kClimbDeployPosition = 0.5;

    public static final double kClimbRetractPosition = 0.5;
}
