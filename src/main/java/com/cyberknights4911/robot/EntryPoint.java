package com.cyberknights4911.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class EntryPoint {
  private EntryPoint() {}

  public static void main(String... args) {
    RobotBase.startRobot(new RobotSupplier());
  }
}
