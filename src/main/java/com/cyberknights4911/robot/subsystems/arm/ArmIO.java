package com.cyberknights4911.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double shoulderPositionDeg = 0.0;
        public double shoulderVelocityDegPerSec = 0.0;
        public double[] shoulderAppliedVolts = new double[] {};
        public double[] shoulderCurrentAmps = new double[] {};
        public double[] shoulderTempCelcius = new double[] {};
        
        public double wristPositionDeg = 0.0;
        public double wristVelocityDegPerSec = 0.0;
        public double wristAppliedVolts = 0.0;
        public double wristCurrentAmps = 0.0;
        public double wristTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    /** Get wrist motor position. */
    public default double getWristPosition() {
        return Double.MIN_VALUE;
    }

    /** Get shoulder motor position. */
    public default double getShoulderPosition() {
        return Double.MIN_VALUE;
    }

    /** Set shoulder motors position. */
    public default void setShoulderPosition(double position) {}
    
    /** Set wrist motor position. */
    public default void setWristPosition(double position) {}
}
