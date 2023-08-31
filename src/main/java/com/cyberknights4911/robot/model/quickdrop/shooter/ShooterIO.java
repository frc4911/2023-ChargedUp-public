package com.cyberknights4911.robot.model.quickdrop.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    class ShooterIOInputs {
        public double velocityRpm = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

    default void setShooterSpeed(double speed) {}

}
