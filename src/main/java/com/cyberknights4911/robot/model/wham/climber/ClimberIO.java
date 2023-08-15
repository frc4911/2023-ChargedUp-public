package com.cyberknights4911.robot.model.wham.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public boolean extended = false;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ClimberIOInputs inputs) {}

    /** Set solenoid state. */
    default void setExtended(boolean extended) {}
    
}
