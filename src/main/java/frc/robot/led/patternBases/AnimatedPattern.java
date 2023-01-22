package frc.robot.led.patternBases;

import frc.robot.drivers.Color;

public class AnimatedPattern extends LEDPattern {
	LEDPattern[] eventList;
	int ticks;
	int tick;

	public AnimatedPattern(){
		super();
		ticks=1;
		eventList = new LEDPattern[ticks];
	}

	public AnimatedPattern(int ticks){
		super();
		this.ticks=ticks;
		eventList = new LEDPattern[ticks];
	}

	@Override
	public Color[] getPattern(){
		tick=(tick+1)%ticks;
		pattern = eventList[tick].getPattern();
		return pattern;
	}

	public void setEvent(int tick, LEDPattern event){
		eventList[tick]=event;
	}
}
