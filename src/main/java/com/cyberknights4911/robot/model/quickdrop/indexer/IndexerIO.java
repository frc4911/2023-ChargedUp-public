package com.cyberknights4911.robot.model.quickdrop.indexer;

public interface IndexerIO {

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
