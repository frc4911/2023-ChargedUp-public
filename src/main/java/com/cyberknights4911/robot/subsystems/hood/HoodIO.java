package com.cyberknights4911.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(HoodIOInputs inputs) {}

    /** Set motor position. */
    public default void setPosition(double position) {}
}
