package com.cyberknights4911.robot.model.quickdrop.shooter;

public interface ShooterIO {

    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

}
