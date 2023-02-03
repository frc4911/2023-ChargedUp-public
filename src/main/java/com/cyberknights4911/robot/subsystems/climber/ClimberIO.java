package com.cyberknights4911.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean extended = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Set solenoid state. */
    public default void setExtended(boolean extended) {}
    
}
