package com.cyberknights4911.robot.model.quickdrop.collector;

import java.util.function.Supplier;

public interface CollectorIO {

    static CollectorIO create(boolean isReal, Supplier<CollectorIO> realCollectorSupplier) {
        if (isReal) {
            return realCollectorSupplier.get();
        } else {
            return new CollectorIO() {};
        }
    }

    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

    default void extend() {}
    
    default void retract() {}
}
