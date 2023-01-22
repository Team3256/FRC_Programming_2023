package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import frc.robot.led.patternBases.LEDPattern;

public class LEDAllSectionSetPattern extends CommandBase{
	private final LED LEDSubsystem;
	private final LEDPattern pattern;

	public LEDAllSectionSetPattern(LED LEDSubsystem, LEDPattern pattern){
		addRequirements(LEDSubsystem);
		this.LEDSubsystem = LEDSubsystem;
		this.pattern = pattern;
	}

	@Override
	public void initialize() {
		LEDSubsystem.bulkSet(pattern);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
