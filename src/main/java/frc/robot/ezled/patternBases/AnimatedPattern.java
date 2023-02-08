// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ezled.patternBases;

public class AnimatedPattern extends LEDPattern {
  private LEDPattern[] keyFrames;
  private final int frames;
  private int currentFrame = 0;

  /**
   * LEDPattern that changes over time
   *
   * @param frames number of frames in the animation
   */
  public AnimatedPattern(int frames) {
    super();
    this.frames = frames;
    keyFrames = new LEDPattern[frames];
  }

  /** Increment current frame and set LEDPattern's pattern to the new pattern */
  @Override
  public void updatePattern() {
    currentFrame = (currentFrame + 1) % frames;
    if (keyFrames[currentFrame] != null) setPattern(keyFrames[currentFrame]);
  }

  /**
   * Set frame of animation
   *
   * @param frame frame that will be set
   * @param event LEDPattern that will be switched to when frame is reached
   */
  protected void createKeyFrame(int frame, LEDPattern event) {
    keyFrames[frame] = event;
  }
}
