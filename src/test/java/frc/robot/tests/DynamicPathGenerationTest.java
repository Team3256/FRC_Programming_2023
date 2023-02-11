// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.helpers.DynamicPathGenerator;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.junit.jupiter.api.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

public class DynamicPathGenerationTest {
	@Test
	public void testGeneratePath(){
		Pose2d src = new Pose2d(new Translation2d(6,3),new Rotation2d(0));
		Pose2d sink = new Pose2d(new Translation2d(2,3), new Rotation2d(0));
		DynamicPathGenerator generator = new DynamicPathGenerator(src, sink);
		List<Pose2d> path = generator.computePath();
		PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
	}

	public void saveJson(JSONObject json){
		try {
			PrintWriter pw = new PrintWriter(new FileWriter("DynamicPathGenerationTest.path"));
			pw.println(json.toJSONString());
			pw.close();
		} catch (IOException e){
			System.out.println("An error occured.");
			e.printStackTrace();
		}
	}

	public JSONObject getJson(List<Pose2d> path){
		JSONObject ret = new JSONObject();

		//waypoints
		JSONArray pathJson = new JSONArray();
		for (int poseIndex=0;poseIndex<path.size();poseIndex++){
			pathJson.add(poseJson(path,poseIndex));
		}
		ret.put("waypoints",pathJson);

		//markers
		JSONArray markerJson = new JSONArray();
		ret.put("markers",markerJson);

		//ret
		return ret;
	}

	public JSONObject poseJson(List<Pose2d> path, int poseIndex){
		JSONObject ret = new JSONObject();
		ret.put("anchorPoint",pointJson(path.get(poseIndex)));
		ret.put("prevControl",poseIndex-1>0?pointJson(path.get(poseIndex-1)):null);
		ret.put("nextControl",poseIndex+1<path.size()?pointJson(path.get(poseIndex+1)):null);
		ret.put("holonomicAngle",path.get(poseIndex).getRotation().getDegrees());
		ret.put("isReversal",false);
		ret.put("velOverride",null);
		ret.put("isLocked",true);
		ret.put("isStopPoint",false);
		return ret;
	}

	public JSONObject pointJson(Pose2d pose){
		JSONObject ret = new JSONObject();
		ret.put("x",pose.getX());
		ret.put("y",pose.getY());
		return ret;
	}
}
