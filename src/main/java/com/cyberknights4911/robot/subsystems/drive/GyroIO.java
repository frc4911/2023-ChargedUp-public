package com.cyberknights4911.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
      public boolean connected = false;
      public double positionRad = 0.0;
      public double velocityRadPerSec = 0.0;
    }
  
    public default void updateInputs(GyroIOInputs inputs) {}

    /** Get the turn rotation */
    public default double getYaw() {
        return 0.0;
    }

    /** Get the pitch rotation */
    public default double getPitch() {
        return 0.0;
    }

    /** Get the roll rotation */
    public default double getRoll() {
        return 0.0;
    }

    /** Reset the sensor yaw rotation */
    public default void setYaw(double angleInDegrees) {}
}
