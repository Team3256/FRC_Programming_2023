package frc.robot.led.patternBases;

import static frc.robot.Constants.LEDConstants.offPattern;

public class BlinkingPattern extends AnimatedPattern{
	public BlinkingPattern(int onTicks, int offTicks, LEDPattern pattern){
		super(onTicks+offTicks);
		setEvent(0,pattern);
		setEvent(onTicks,offPattern);
	}
}
