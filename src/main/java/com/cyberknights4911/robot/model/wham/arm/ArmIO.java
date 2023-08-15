package com.cyberknights4911.robot.model.wham.arm;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    static ArmIO create(boolean isReal, Supplier<ArmIO> realArmSupplier) {
      if (isReal) {
        return realArmSupplier.get();
      } else {
        return new ArmIO() {};
      }
    }

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
    default void updateInputs(ArmIOInputs inputs) {}

    /** Get wrist encoder degrees. */
    default double getWristEncoderDegrees() {
        return 0.0;
    }

    /** Get shoulder encoder degrees. */
    default double getShoulderEncoderDegrees() {
        return 0.0;
    }

    default boolean isWristEncoderConnected() {
        return true;
    }

    default boolean isShoulderEncoderConnected() {
        return true;
    }

    default void setShoulderBrakeMode() {}

    default void setWristBrakeMode() {}

    /** Set shoulder motors output. */
    default void setShoulderOutput(double output) {}
    
    /** Set wrist motor output. */
    default void setWristOutput(double output) {}

    /** Set shoulder motors position. */
    default void setShoulderPosition(double position) {}
    
    /** Set wrist motor position. */
    default void setWristPosition(double position) {}

    default double offsetWrist() { return 0;}

    default double offsetShoulder() {return 0;}

    default double getShoulderTrajectoryPosition() { return 0.0; }

    default double getWristTrajectoryPosition() { return 0.0; }
}
