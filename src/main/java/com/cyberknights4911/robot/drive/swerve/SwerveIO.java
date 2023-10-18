package com.cyberknights4911.robot.drive.swerve;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public interface SwerveIO {

    @AutoLog
    class SwerveIOInputs {
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
    void updateInputs(SwerveIOInputs inputs);

    /** Reset the angle encoder to absoloute. */
    void resetToAbsolute();
  
    /** Set the turn motor to the specified ControlMode and value. */
    void setAngle(ControlMode mode, double outputValue);

    /** Set the drive motor to the specified ControlMode and value. */
    void setDrive(ControlMode mode, double outputValue);
    
    /** Set the drive motor to the specified ControlMode, output value, DemandType and demandValue */
    void setDrive(
        ControlMode mode, double outputValue, DemandType demandType, double demandValue
    );
    
    /** Get drive motor position. */
    double getDriveSensorPosition();

    /** Get angle motor position. */
    double getAngleSensorPosition();
    
    /** Get drive motor velocity. */
    double getDriveSensorVelocity();

    double getAngleEncoderDegrees();
}
  