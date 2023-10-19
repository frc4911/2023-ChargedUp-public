package com.cyberknights4911.robot.model.wham.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    class ArmIOInputs {
        public double shoulderPositionDeg = 0.0;
        public double shoulderSelectedSensorPosition = 0.0;
        public double shoulderRemoteEncoderPosition = 0.0;
        public double shoulderVelocityUnitsPerHundredMs = 0.0;
        public double[] shoulderAppliedVolts = new double[] {};
        public double[] shoulderCurrentAmps = new double[] {};
        public double[] shoulderTempCelcius = new double[] {};
        
        public double wristPositionDeg = 0.0;
        public double wristSelectedSensorPosition = 0.0;
        public double wristRemoteEncoderPosition = 0.0;
        public double wristVelocityDegPerSec = 0.0;
        public double wristAppliedVolts = 0.0;
        public double wristCurrentAmps = 0.0;
        public double wristTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    void updateInputs(ArmIOInputs inputs);

    /** Get wrist encoder degrees. */
    double getWristEncoderDegrees();

    /** Get shoulder encoder degrees. */
    double getShoulderEncoderDegrees();

    /** Set shoulder motors position. */
    void setShoulderPosition(double position);
    
    /** Set wrist motor position. */
    void setWristPosition(double position);

    double getShoulderTrajectoryPosition();

    double getWristTrajectoryPosition();
}
