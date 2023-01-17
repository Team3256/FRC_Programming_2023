package frc.robot.drivers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTrigger extends Trigger {
    public JoystickTrigger(){

    }
    //TODO: Create a gradle task that when run will print the command name
    @Override
    public Trigger whileTrue(Command command) {
        super.whileTrue(command);
        System.out.println(command.getName());
        return this;
    }
}
