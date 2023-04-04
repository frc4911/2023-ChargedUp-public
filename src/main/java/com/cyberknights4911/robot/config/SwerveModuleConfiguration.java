package com.cyberknights4911.robot.config;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class SwerveModuleConfiguration {
    public static final int STEER_MOTOR_SLOT_0_I_ZONE = 25;

    public static final SensorInitializationStrategy CAN_CODER_SENSOR_INITIALIZATION_STRATEGY =
            SensorInitializationStrategy.BootToAbsolutePosition;
    // Steer Motor measurement
    // dt for velocity measurements, ms
    public static final SensorVelocityMeasPeriod STEER_MOTOR_VELOCITY_MEASUREMENT_PERIOD =
            SensorVelocityMeasPeriod.Period_100Ms;
    public static final int STEER_MOTOR_VELOCITY_MEASUREMENT_WINDOW = 64; // # of samples in rolling average

    public final String name;
    public final int moduleId = -1;
    public final int driveMotorTalonId;
    public final int steerMotorTalonId;
    public final double steerReduction;
    public final double steerTicksPerUnitDistance;
    public final double steerTicksPerUnitVelocity;

    // general Steer Motor
    public final boolean invertSteerMotor;
    public final double steerMotorTicksPerRadian;

    // Steer CANCoder
    public final int CANCoderId;
    public final double CANCoderOffsetDegrees;

    // Steer Motor motion
    public final double steerMotorSlot0Kp;
    public final double steerMotorSlot0Ki;
    public final double steerMotorSlot0Kd;
    public final double steerMotorSlot0Kf;

    // general drive
    public final boolean invertDrive;
    // Default wheel diameter and drive reduction to Mk4_L2i values which are in SI
    // units
    public final double wheelDiameter;
    public final double driveReduction;
    public final double driveTicksPerUnitDistance;
    public final double driveTicksPerUnitVelocity;
    public final double driveDeadband = 0.01;

    private SwerveModuleConfiguration(String name, int driveMotorTalonId, int steerMotorTalonId, int CANCoderId,
            double CANCoderOffsetDegrees, double wheelDiameter, double driveReduction, double steerReduction,
            boolean invertDrive, boolean invertSteerMotor, double steerMotorSlot0Kp, double steerMotorSlot0Ki,
            double steerMotorSlot0Kd, double steerMotorSlot0Kf) {
        this.name = name;
        this.driveMotorTalonId = driveMotorTalonId;
        this.steerMotorTalonId = steerMotorTalonId;
        this.CANCoderId = CANCoderId;
        this.CANCoderOffsetDegrees = CANCoderOffsetDegrees;
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.steerReduction = steerReduction;
        this.invertDrive = invertDrive;
        this.invertSteerMotor = invertSteerMotor;
        this.steerMotorSlot0Kp = steerMotorSlot0Kp;
        this.steerMotorSlot0Ki = steerMotorSlot0Ki;
        this.steerMotorSlot0Kd = steerMotorSlot0Kd;
        this.steerMotorSlot0Kf = steerMotorSlot0Kf;

        steerTicksPerUnitDistance = (1.0 / 2048.0) * steerReduction * (2.0 * Math.PI);
        steerTicksPerUnitVelocity = steerTicksPerUnitDistance * 10; // Motor controller unit is ticks per 100 ms
        steerMotorTicksPerRadian = (2048.0 / steerReduction) / (2.0 * Math.PI); // for steer motor
        driveTicksPerUnitDistance = (1.0 / 2048.0) * driveReduction * (Math.PI * wheelDiameter);
        driveTicksPerUnitVelocity = driveTicksPerUnitDistance * 10; // Motor controller unit is ticks per 100 ms
    }

    public static class Builder {
        private String name;
        private int driveMotorTalonId;
        private int steerMotorTalonId;
        private int canCoderId;
        private double canCoderOffsetDegrees;
        private double wheelDiameter;
        private double driveReduction;
        private double steerReduction;
        private boolean invertDrive;
        private boolean invertSteerMotor;
        private double steerMotorSlot0Kp;
        private double steerMotorSlot0Ki;
        private double steerMotorSlot0Kd;
        private double steerMotorSlot0Kf;

        public Builder setName(String name) {
            this.name = name;
            return this;
        }

        public Builder setDriveMotorTalonId(int driveMotorTalonId) {
            this.driveMotorTalonId = driveMotorTalonId;
            return this;
        }

        public Builder setSteerMotorTalonId(int steerMotorTalonId) {
            this.steerMotorTalonId = steerMotorTalonId;
            return this;
        }

        public Builder setCANCoderId(int canCoderId) {
            this.canCoderId = canCoderId;
            return this;
        }

        public Builder setCANCoderOffsetDegrees(double canCoderOffsetDegrees) {
            this.canCoderOffsetDegrees = canCoderOffsetDegrees;
            return this;
        }

        public Builder setWheelDiameter(double wheelDiameter) {
            this.wheelDiameter = wheelDiameter;
            return this;
        }

        public Builder setDriveReduction(double driveReduction) {
            this.driveReduction = driveReduction;
            return this;
        }

        public Builder setSteerReduction(double steerReduction) {
            this.steerReduction = steerReduction;
            return this;
        }

        public Builder setInvertDrive(boolean invertDrive) {
            this.invertDrive = invertDrive;
            return this;
        }

        public Builder setInvertSteerMotor(boolean invertSteerMotor) {
            this.invertSteerMotor = invertSteerMotor;
            return this;
        }

        public Builder setSteerMotorSlot0Kp(double steerMotorSlot0Kp) {
            this.steerMotorSlot0Kp = steerMotorSlot0Kp;
            return this;
        }

        public Builder setSteerMotorSlot0Ki(double steerMotorSlot0Ki) {
            this.steerMotorSlot0Ki = steerMotorSlot0Ki;
            return this;
        }

        public Builder setSteerMotorSlot0Kd(double steerMotorSlot0Kd) {
            this.steerMotorSlot0Kd = steerMotorSlot0Kd;
            return this;
        }

        public Builder setSteerMotorSlot0Kf(double steerMotorSlot0Kf) {
            this.steerMotorSlot0Kf = steerMotorSlot0Kf;
            return this;
        }

        public SwerveModuleConfiguration build() {
            return new SwerveModuleConfiguration(name, driveMotorTalonId, steerMotorTalonId, canCoderId,
                    canCoderOffsetDegrees, wheelDiameter, driveReduction, steerReduction, invertDrive, invertSteerMotor,
                    steerMotorSlot0Kp, steerMotorSlot0Ki, steerMotorSlot0Kd, steerMotorSlot0Kf);
        }
    }
}
