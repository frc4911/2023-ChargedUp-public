package com.cyberknights4911.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.ControlMode;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
      public double drivePositionRad = 0.0;
      public double driveVelocityRadPerSec = 0.0;
      public double driveVelocityFilteredRadPerSec = 0.0;
      public double driveAppliedVolts = 0.0;
      public double driveCurrentAmps = 0.0;
      public double driveTempCelcius = 0.0;
  
      public double turnAbsolutePositionRad = 0.0;
      public double turnPositionRad = 0.0;
      public double turnVelocityRadPerSec = 0.0;
      public double turnAppliedVolts = 0.0;
      public double turnCurrentAmps = 0.0;
      public double turnTempCelcius = 0.0;
    }
  
    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveIOInputs inputs) {}
  
    /** Set the turn motor to the specified ControlMode and value. */
    public default void setTurn(ControlMode mode, double outputValue) {}

    /** Set the drive motor to the specified ControlMode and value. */
    public default void setDrive(ControlMode mode, double outputValue) {}
    
    /** Get drive motor position. */
    public default double getDrivePosition() {
        return Double.MIN_VALUE;
    }

    /** Get turn motor position. */
    public default double getTurnPosition() {
        return Double.MIN_VALUE;
    }
    
    /** Get drive motor velocity. */
    public default double getDriveVelocity() {
        return Double.MIN_VALUE;
    }

    /** Get turn motor velocity. */
    public default double getTurnVelocity() {
        return Double.MIN_VALUE;
    }
  
    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}
  
    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(boolean enable) {}

    /** Get the turn motor sensor position. */
    public default double getTurnSensorPosition() {
        return Double.MIN_VALUE;
    }

    /** Set the turn motor sensor position. */
    public default void setTurnSensorPosition(double value) {}

    /** Get the timestamp from the encoder. */
    public default double getLastTimeStamp() {
        return Double.MIN_VALUE;
    }

    /** Get the absolute position from the encoder. */
    public default double getAbsolutePosition() {
        return Double.MIN_VALUE;
    }
  }
  