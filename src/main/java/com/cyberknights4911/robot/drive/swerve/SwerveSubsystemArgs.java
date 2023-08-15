package com.cyberknights4911.robot.drive.swerve;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class SwerveSubsystemArgs {
    public abstract GyroIO gyroIO();
    public abstract SwerveModule frontLeftSwerveModule();
    public abstract SwerveModule frontRightSwerveModule();
    public abstract SwerveModule backLeftSwerveModule();
    public abstract SwerveModule backRightSwerveModule();
    public abstract SwerveDriveConstants swerveDriveConstants();

    public static Builder builder() {
        return new AutoValue_SwerveSubsystemArgs.Builder()
            .setGyroIO(new GyroIO() {});
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder setGyroIO(GyroIO value);
        public abstract Builder setFrontLeftSwerveModule(SwerveModule value);
        public abstract Builder setFrontRightSwerveModule(SwerveModule value);
        public abstract Builder setBackLeftSwerveModule(SwerveModule value);
        public abstract Builder setBackRightSwerveModule(SwerveModule value);
        public abstract Builder setSwerveDriveConstants(SwerveDriveConstants value);
        public abstract SwerveSubsystemArgs build();
    }
}
