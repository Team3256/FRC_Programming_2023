package frc.robot.led.patterns;

import frc.robot.led.patternBases.AnimatedPattern;

public class SpiritPattern extends AnimatedPattern {
	public SpiritPattern(){
		super(100);
		setEvent(0,new ConePattern());
		setEvent(50,new CubePattern());
	}
}
