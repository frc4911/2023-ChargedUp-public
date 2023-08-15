package com.cyberknights4911.robot.model.wham.slurpp;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

public interface SlurppIO {
  static SlurppIO create(boolean isReal, Supplier<SlurppIO> realSlurppSupplier) {
    if (isReal) {
      return realSlurppSupplier.get();
    } else {
      return new SlurppIO() {};
    }
  }

    @AutoLog
    class SlurppIOInputs {
      public double positionDeg = 0.0;
      public double velocityRpm = 0.0;
      public double appliedVolts = 0.0;
      public double currentAmps = 0.0;
      public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(SlurppIOInputs inputs) {}

    /** Set motor percent output. */
    default void setPercentOutput(double percentOutput) {}

    /** Stop the motor. */
    default void stop() {}

    /** Hold current position. */
    default void holdCurrentPosition() {}
}
