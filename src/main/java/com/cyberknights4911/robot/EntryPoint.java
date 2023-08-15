package com.cyberknights4911.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class EntryPoint {
  private EntryPoint() {}

  public static void main(String... args) {
    RobotSupplier robotSupplier = new RobotSupplier();
    RobotBase.startRobot(robotSupplier::get);
  }
}
