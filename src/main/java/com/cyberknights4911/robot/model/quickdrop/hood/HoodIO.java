package com.cyberknights4911.robot.model.quickdrop.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(HoodIOInputs inputs) {}

    /** Set motor position. */
    default void setPosition(double position) {}
}
