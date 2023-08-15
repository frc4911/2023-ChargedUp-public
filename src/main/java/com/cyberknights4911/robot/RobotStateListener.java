package com.cyberknights4911.robot;

import org.littletonrobotics.junction.LoggedRobot;

public interface RobotStateListener {
    default void onAutonomousInit(LoggedRobot robot) {}

    default void onAutonomousExit(LoggedRobot robot) {}

    default void onTeleopInit(LoggedRobot robot) {}

    default void onTeleopExit(LoggedRobot robot) {}
}
