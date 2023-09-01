package com.cyberknights4911.robot.model.quickdrop.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public double velocityRpm = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets the shooter motor speed. */
    default void setShooterSpeed(double speed) {}
    
    /** Sets the hood position as a percent (zero is all the way retracted). */
    default void setHoodPosition(double percent) {}
}
