package com.cyberknights4911.robot.drive.swerve;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class SwerveModuleArgs {
    public abstract int moduleNumber();
    public abstract SwerveIO swerveIO();
    public abstract SwerveDriveConstants swerveDriveConstants();
    public abstract CotsFalconSwerveConstants cotsConstants();

    public static Builder builder() {
        return new AutoValue_SwerveModuleArgs.Builder()
            .setSwerveIO(new SwerveIO() {});
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder setModuleNumber(int value);
        public abstract Builder setSwerveIO(SwerveIO value);
        public abstract Builder setSwerveDriveConstants(SwerveDriveConstants value);
        public abstract Builder setCotsConstants(CotsFalconSwerveConstants value);
        public abstract SwerveModuleArgs build();
    }
}
