// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.dynamicpathgeneration.helpers;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.json.simple.JSONObject;

public class FileHelper {
  public static void saveJson(JSONObject json, String path) {
    try {
      PrintWriter pw = new PrintWriter(new FileWriter(path));

      json.writeJSONString(pw);
      pw.close();
    } catch (IOException e) {
      System.out.println("IO error occurred.");
      e.printStackTrace();
    }
  }
}
