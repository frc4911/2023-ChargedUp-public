package com.cyberknights4911.robot.drive.swerve;

import com.google.auto.value.AutoValue;

/**
 * Holds constants related to individual swerve modules.
 * For overall swerve drive values, use {@link SwerveDriveConstants}.
 * For manufacturer specific values, use {@link CotsFalconSwerveConstants}.
 */
@AutoValue
public abstract class SwerveModuleConstants {
    public abstract int driveMotorId();
    public abstract int angleMotorId();
    public abstract int canCoderId();
    public abstract double angleOffsetDegrees();

    public static Builder builder() {
        return new AutoValue_SwerveModuleConstants.Builder();
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder setDriveMotorId(int value);
        public abstract Builder setAngleMotorId(int value);
        public abstract Builder setCanCoderId(int value);
        public abstract Builder setAngleOffsetDegrees(double value);
        public abstract SwerveModuleConstants build();
    }
}
