package com.cyberknights4911.robot.drive.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.auto.value.AutoValue;

@AutoValue
public abstract class SwerveDriveConstants {
    /* Robot dimensions */
    public abstract double trackWidth();
    public abstract double wheelBase();

    /* Meters per Second */
    public abstract double maxSpeed();
    /* Radians per Second */
    public abstract double maxAngularVelocity();

    public abstract double stickDeadband();

    /* Drive Motor Gain Values */
    public abstract double driveStaticGain();
    public abstract double driveVelocityGain();
    public abstract double driveAccelerationGain();
    public abstract double driveProportionalGain();
    public abstract double driveIntegralGain();
    public abstract double driveDerivativeGain();
    public abstract double driveFeedForwardGain();

    public abstract int gyroId();
    public abstract boolean invertGyro();

    /* Used to ramp in open loop and closed loop driving */
    public abstract double openloopRamp();
    public abstract double closedloopRamp();

    /* Neutral Modes */
    public abstract NeutralMode angleNeutralMode();
    public abstract NeutralMode driveNeutralMode();

    /* Swerve Angle Current Limiting */
    public abstract int angleContinuousCurrentLimit();
    public abstract int anglePeakCurrentLimit();
    public abstract double anglePeakCurrentDuration();
    public abstract boolean angleEnableCurrentLimit();

    /* Swerve Drive Current Limiting */
    public abstract int driveContinuousCurrentLimit();
    public abstract int drivePeakCurrentLimit();
    public abstract double drivePeakCurrentDuration();
    public abstract boolean driveEnableCurrentLimit();
    
    public static Builder builder() {
        return new AutoValue_SwerveDriveConstants.Builder();
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder setTrackWidth(double value);
        public abstract Builder setWheelBase(double value);
        public abstract Builder setMaxSpeed(double value);
        public abstract Builder setMaxAngularVelocity(double value);
        public abstract Builder setStickDeadband(double value);
        public abstract Builder setGyroId(int value);
        public abstract Builder setInvertGyro(boolean value);
        public abstract Builder setOpenloopRamp(double value);
        public abstract Builder setClosedloopRamp(double value);
        public abstract Builder setDriveProportionalGain(double value);
        public abstract Builder setDriveIntegralGain(double value);
        public abstract Builder setDriveDerivativeGain(double value);
        public abstract Builder setDriveFeedForwardGain(double value);
        public abstract Builder setDriveStaticGain(double value);
        public abstract Builder setDriveVelocityGain(double value);
        public abstract Builder setDriveAccelerationGain(double value);
        public abstract Builder setAngleNeutralMode(NeutralMode value);
        public abstract Builder setDriveNeutralMode(NeutralMode value);
        public abstract Builder setAngleContinuousCurrentLimit(int value);
        public abstract Builder setAnglePeakCurrentLimit(int value);
        public abstract Builder setAnglePeakCurrentDuration(double value);
        public abstract Builder setAngleEnableCurrentLimit(boolean value);
        public abstract Builder setDriveContinuousCurrentLimit(int value);
        public abstract Builder setDrivePeakCurrentLimit(int value);
        public abstract Builder setDrivePeakCurrentDuration(double value);
        public abstract Builder setDriveEnableCurrentLimit(boolean value);
        public abstract SwerveDriveConstants build();
    }
}
