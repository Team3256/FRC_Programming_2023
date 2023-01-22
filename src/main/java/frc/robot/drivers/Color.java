// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

/** class that holds the state of a RGB color some helper methods implemented */
public class Color {
  public int R;
  public int G;
  public int B;

  public Color(int R, int G, int B) {
    set(R, G, B);
  }

  public Color() {
    set(0, 0, 0);
  }

  public void set(Color color) {
    set(color.R, color.G, color.B);
  }

  public void set(int R, int G, int B) {
    if (!inColorRange(R) || !inColorRange(G) || !inColorRange(B)) return;
    this.R = R;
    this.G = G;
    this.B = B;
  }

  public boolean inColorRange(int value) {
    return !(value < 0 || value > 255);
  }

  public String toString() {
    return "(R:" + R + ", G:" + G + ", B:" + B + ")";
  }
}
