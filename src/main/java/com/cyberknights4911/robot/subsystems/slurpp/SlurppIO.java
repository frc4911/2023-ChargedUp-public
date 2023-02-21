package com.cyberknights4911.robot.subsystems.slurpp;

import org.littletonrobotics.junction.AutoLog;

public interface SlurppIO {
    @AutoLog
    public static class SplurppIOInputs {
      public double positionDeg = 0.0;
      public double velocityRpm = 0.0;
      public double appliedVolts = 0.0;
      public double currentAmps = 0.0;
      public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SplurppIOInputs inputs) {}

    /** Set motor percent output. */
    public default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    public default void stop() {}
}
