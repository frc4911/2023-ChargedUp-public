package com.cyberknights4911.robot.subsystems.bob;

import org.littletonrobotics.junction.AutoLog;

public interface BobIO {
    @AutoLog
    public static class BobIOInputs {
        public boolean extended = false;

        public double positionDeg = 0.0;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(BobIOInputs inputs) {}

    /** Set solenoid state. */
    public default void setExtended(boolean extended) {}

    /** Set motor position. */
    public default void setPosition(double position) {}
    
}
