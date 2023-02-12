package frc.robot.auto.dynamicpathgeneration.helpers;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;

public class Obstacle{
	private Translation2d[] points;
	public Obstacle(Translation2d[] points){
		this.points=points;
	}
	public boolean containsPoint(Translation2d query){
		Translation2d wild = new Translation2d(9999,9999);
		int intersections = 0;
		for (int i=0;i<points.length;i++){
			int j = (i+1)%points.length;
			if (GeometryUtil.intersect(query,wild,points[i],points[j])) intersections++;
		}
		return intersections%2==1;
	}
}
