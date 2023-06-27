package com.cyberknights4911.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double drivePositionDeg = 0.0;
        public double driveVelocityRpm = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;
    
        public double turnAbsolutePositionDeg = 0.0;
        public double turnPositionDeg = 0.0;
        public double turnVelocityRpm = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelcius = 0.0;
    }
  
    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveIOInputs inputs) {}

    /** Reset the angle encoder to absoloute. */
    public default void resetToAbsolute() {}
  
    /** Set the turn motor to the specified ControlMode and value. */
    public default void setAngle(ControlMode mode, double outputValue) {}

    /** Set the drive motor to the specified ControlMode and value. */
    public default void setDrive(ControlMode mode, double outputValue) {}
    
    /** Set the drive motor to the specified ControlMode, output value, DemandType and demandValue */
    public default void setDrive(
        ControlMode mode, double outputValue, DemandType demandType, double demandValue
    ) {}
    
    /** Get drive motor position. */
    public default double getDriveSensorPosition() {
        return Double.MIN_VALUE;
    }

    /** Get angle motor position. */
    public default double getAngleSensorPosition() {
        return Double.MIN_VALUE;
    }
    
    /** Get drive motor velocity. */
    public default double getDriveSensorVelocity() {
        return Double.MIN_VALUE;
    }

    public default double getAngleEncoderDegrees() {
        return Double.MIN_VALUE;
    }
}
  