package com.cyberknights4911.robot.drive.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
      public boolean connected = false;
      public double positionRad = 0.0;
      public double velocityRadPerSec = 0.0;
    }
  
    void updateInputs(GyroIOInputs inputs);

    /** Get the turn rotation */
    double getYaw();

    /** Get the pitch rotation */
    double getPitch();

    /** Get the roll rotation */
    double getRoll();

    /** Reset the sensor yaw rotation */
    void setYaw(double angleInDegrees);
}
