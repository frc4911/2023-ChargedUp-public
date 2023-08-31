package com.cyberknights4911.robot.model.quickdrop.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerIOInputs {
        public double enterVoltage = 0.0;
        public double exitVoltage = 0.0;
        public double velocityRpm = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}
    
    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

    default boolean isEnterBlocked() {
        return false;
    }

    default boolean isExitBlocked() {
        return false;
    }

}
