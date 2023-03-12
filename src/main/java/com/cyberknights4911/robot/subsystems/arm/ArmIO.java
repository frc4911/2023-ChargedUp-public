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

    /** Get wrist encoder degrees. */
    public default double getWristEncoderDegrees() {
        return 0.0;
    }

    /** Get shoulder encoder degrees. */
    public default double getShoulderEncoderDegrees() {
        return 0.0;
    }

    public default boolean isWristEncoderConnected() {
        return true;
    }

    public default boolean isShoulderEncoderConnected() {
        return true;
    }

    public default void setShoulderBrakeMode() {}

    public default void setWristBrakeMode() {}

    /** Set shoulder motors output. */
    public default void setShoulderOutput(double output) {}
    
    /** Set wrist motor output. */
    public default void setWristOutput(double output) {}

    /** Set shoulder motors position. */
    public default void setShoulderPosition(double position) {}
    
    /** Set wrist motor position. */
    public default void setWristPosition(double position) {}

    public default void adjustError() {}
}
