package com.cyberknights4911.robot.drive.swerve;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public interface SwerveIO {
    static SwerveIO create(boolean isReal, Supplier<SwerveIO> realSwerveSupplier) {
        if (isReal) {
            return realSwerveSupplier.get();
        } else {
            return new SwerveIO() {};
        }
    }

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
    default void updateInputs(SwerveIOInputs inputs) {}

    /** Reset the angle encoder to absoloute. */
    default void resetToAbsolute() {}
  
    /** Set the turn motor to the specified ControlMode and value. */
    default void setAngle(ControlMode mode, double outputValue) {}

    /** Set the drive motor to the specified ControlMode and value. */
    default void setDrive(ControlMode mode, double outputValue) {}
    
    /** Set the drive motor to the specified ControlMode, output value, DemandType and demandValue */
    default void setDrive(
        ControlMode mode, double outputValue, DemandType demandType, double demandValue
    ) {}
    
    /** Get drive motor position. */
    default double getDriveSensorPosition() {
        return Double.MIN_VALUE;
    }

    /** Get angle motor position. */
    default double getAngleSensorPosition() {
        return Double.MIN_VALUE;
    }
    
    /** Get drive motor velocity. */
    default double getDriveSensorVelocity() {
        return Double.MIN_VALUE;
    }

    default double getAngleEncoderDegrees() {
        return Double.MIN_VALUE;
    }
}
  