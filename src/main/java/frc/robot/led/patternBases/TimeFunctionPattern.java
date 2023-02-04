package frc.robot.led.patternBases;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.LEDConstants.kResolution;

public abstract class TimeFunctionPattern extends LEDPattern{
	private int ticks = 0;
	//determines whether to render and animate the color as rgb or hsv
	boolean rgb;
	public TimeFunctionPattern(boolean rgb) {
		this.rgb=rgb;
	}
	//Red/Hue [0,1) as a function of ticks
	abstract double calculateA(int ticks);
	//Green/Saturation [0,1] as a function of ticks
	abstract double calculateB(int ticks);
	//Blue/Value [0,1] as a function of ticks
	abstract double calculateC(int ticks);

	@Override
	public void update(){
		for (int pixel=1;pixel<=kResolution;pixel++){
			double A = calculateA(ticks+pixel);
			double B = calculateB(ticks+pixel);
			double C = calculateC(ticks+pixel);

			if (rgb){
				setPixel(pixel,new Color(A,B,C));
			} else {
				setPixel(pixel,Color.fromHSV((int)(A*360),(int)(B*255),(int)(C*255)));
			}
		}
		ticks++;
	}
}
