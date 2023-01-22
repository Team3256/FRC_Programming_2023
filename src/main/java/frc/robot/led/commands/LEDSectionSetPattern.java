package frc.robot.led.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LED;
import frc.robot.led.patternBases.LEDPattern;

public class LEDSectionSetPattern extends CommandBase{
	private final LED LEDSubsystem;
	private final LEDPattern pattern;
	int sectionID;

	public LEDSectionSetPattern(LED LEDSubsystem, LEDPattern pattern, int sectionID){
		addRequirements(LEDSubsystem);
		this.LEDSubsystem = LEDSubsystem;
		this.pattern=pattern;
		this.sectionID=sectionID;
	}

	@Override
	public void initialize() {
		LEDSubsystem.set(sectionID, pattern);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
