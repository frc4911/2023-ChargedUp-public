package com.cyberknights4911.robot.subsystems.collector;

public interface CollectorIO {

    /** Set motor percent output. */
    public default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    public default void stop() {}

    public default void extend() {}
    
    public default void retract() {}
}
