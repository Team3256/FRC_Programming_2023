package frc.robot.led.patternBases;

public class ColorChaseBluePattern extends TimeFunctionPattern{
	public ColorChaseBluePattern(){
		super(true);
	}

	private final double cycleTicks = 20;

	@Override
	double calculateA(int ticks){
		return 0.5+0.5*Math.cos(ticks/cycleTicks);
	}

	@Override
	double calculateB(int ticks){
		return 0.5+0.5*Math.cos(ticks/cycleTicks);
	}

	@Override
	double calculateC(int ticks){
		return 255;
	}
}
