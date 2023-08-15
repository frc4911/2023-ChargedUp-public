package com.cyberknights4911.robot;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * Listener interface for receiving callbacks when the robot enters different states.
 */
public interface RobotStateListener {
    default void onAutonomousInit(LoggedRobot robot) {}

    default void onAutonomousExit(LoggedRobot robot) {}

    default void onTeleopInit(LoggedRobot robot) {}

    default void onTeleopExit(LoggedRobot robot) {}
}
