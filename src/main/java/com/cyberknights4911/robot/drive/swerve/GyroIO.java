package com.cyberknights4911.robot.drive.swerve;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    static GyroIO create(boolean isReal, Supplier<GyroIO> realGyroSupplier) {
        if (isReal) {
            return realGyroSupplier.get();
        } else {
            return new GyroIO() {};
        }
    }

    @AutoLog
    class GyroIOInputs {
      public boolean connected = false;
      public double positionRad = 0.0;
      public double velocityRadPerSec = 0.0;
    }
  
    default void updateInputs(GyroIOInputs inputs) {}

    /** Get the turn rotation */
    default double getYaw() {
        return 0.0;
    }

    /** Get the pitch rotation */
    default double getPitch() {
        return 0.0;
    }

    /** Get the roll rotation */
    default double getRoll() {
        return 0.0;
    }

    /** Reset the sensor yaw rotation */
    default void setYaw(double angleInDegrees) {}
}
