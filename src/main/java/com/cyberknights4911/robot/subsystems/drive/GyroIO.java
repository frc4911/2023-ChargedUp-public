package com.cyberknights4911.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
      public boolean connected = false;
      public double positionRad = 0.0;
      public double velocityRadPerSec = 0.0;
    }
  
    public default void updateInputs(GyroIOInputs inputs) {}

    /** Get the turn rotation */
    public default Rotation2d getYaw() {
        return new Rotation2d();
    }

    /** Get the pitch rotation */
    public default Rotation2d getPitch() {
        return new Rotation2d();
    }

    /** Get the roll rotation */
    public default Rotation2d getRoll() {
        return new Rotation2d();
    }

    /** Reset the sensor angle. */
    public default void setAngle(double angleInDegrees) {}
}
