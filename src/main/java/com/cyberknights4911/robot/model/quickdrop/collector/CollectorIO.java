package com.cyberknights4911.robot.model.quickdrop.collector;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorIO {

    @AutoLog
    class CollectorIOInputs {
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
        public boolean isExtended = false;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(CollectorIOInputs inputs) {}

    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

    default void extend() {}
    
    default void retract() {}
}
