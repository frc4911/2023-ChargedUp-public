package com.cyberknights4911.robot.config;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConfiguration {
    public final double wheelbaseLengthInMeters;
    public final double wheelbaseWidthInMeters;
    public final double maxSpeedInMetersPerSecond;
    public final double maxAccellerationInMetersPerSecondSq;
    public final double maxCentriptalAccelerationInMetersPerSecondSq;
    public final double maxSpeedInRadiansPerSecond;
    public final List<Translation2d> moduleLocations;
    // imu heading PID constants
    public final double swerveHeadingKp;
    public final double swerveHeadingKi;
    public final double swerveHeadingKd;
    public final double swerveHeadingKf;
    // public TrajectoryConfig trajectoryConfig;

    /**
     * Creates an instance of SwerveConfiguration
     * <p>
     *
     * @param wheelbaseLengthInMeters         Length in meters of distance between
     *                                        front and back wheels
     * @param wheelbaseWidthInMeters          Length in meters of distance between
     *                                        left and right wheels
     * @param maxSpeedInMetersPerSecond       Max translation speed in meters
     * @param maxSpeedInRadiansPerSecondLimit Max rotation speed in radians per
     *                                        second. Note translation speed
     *                                        and wheelbase geometry may limit
     *                                        rotations speed. If set 0.0, then
     *                                        translation speed and wheelbase
     *                                        geometry determines max rotation
     *                                        speed.
     */
    private SwerveConfiguration(double wheelbaseLengthInMeters,
            double wheelbaseWidthInMeters,
            double maxSpeedInMetersPerSecond,
            double maxSpeedInRadiansPerSecondLimit,
            double swerveHeadingKp,
            double swerveHeadingKi,
            double swerveHeadingKd,
            double swerveHeadingKf) {
        this.swerveHeadingKp = swerveHeadingKp;
        this.swerveHeadingKi = swerveHeadingKi;
        this.swerveHeadingKd = swerveHeadingKd;
        this.swerveHeadingKf = swerveHeadingKf;
        this.wheelbaseLengthInMeters = wheelbaseLengthInMeters;
        this.wheelbaseWidthInMeters = wheelbaseWidthInMeters;
        this.maxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        // TODO - make thsi a parameter. For now default to max speed in 0.5s.
        this.maxAccellerationInMetersPerSecondSq = maxSpeedInMetersPerSecond / 0.5;
        var radiusInMeters = Math.hypot(this.wheelbaseLengthInMeters / 2, this.wheelbaseWidthInMeters / 2);
        this.maxSpeedInRadiansPerSecond = maxSpeedInRadiansPerSecondLimit > 0.0
                ? Math.min((this.maxSpeedInMetersPerSecond / radiusInMeters),
                        maxSpeedInRadiansPerSecondLimit)
                : (this.maxSpeedInMetersPerSecond / radiusInMeters);

        // TODO: Verify this is correct
        this.maxCentriptalAccelerationInMetersPerSecondSq = Math.pow(this.maxSpeedInMetersPerSecond, 2)
                / radiusInMeters;

        // CCW: left positive, right negative, front positive, back negative
        Translation2d kFrontRightModuleLocation = new Translation2d(wheelbaseLengthInMeters / 2,
                -wheelbaseWidthInMeters / 2);
        Translation2d kFrontLeftModuleLocation = new Translation2d(wheelbaseLengthInMeters / 2,
                wheelbaseWidthInMeters / 2);
        Translation2d kBackLeftModuleLocation = new Translation2d(-wheelbaseLengthInMeters / 2,
                wheelbaseWidthInMeters / 2);
        Translation2d kBackRightModuleLocation = new Translation2d(-wheelbaseLengthInMeters / 2,
                -wheelbaseWidthInMeters / 2);

        moduleLocations = Arrays.asList(
                kFrontRightModuleLocation, kFrontLeftModuleLocation, kBackLeftModuleLocation,
                kBackRightModuleLocation);

        // trajectoryConfig = new TrajectoryConfig(
        //         this.maxSpeedInMetersPerSecond, this.maxAccellerationInMetersPerSecondSq,
        //         this.maxSpeedInRadiansPerSecond, this.kMaxCentriptalAccelerationInMetersPerSecondSq)
        //         .addConstraint(new CentripetalAccelerationConstraint(this.kMaxCentriptalAccelerationInMetersPerSecondSq));

        }

    public static final class Builder {
        private double wheelbaseLengthInMeters;
        private double wheelbaseWidthInMeters;
        private double maxSpeedInMetersPerSecond;
        private double maxSpeedInRadiansPerSecondLimit;
        private double swerveHeadingKp;
        private double swerveHeadingKi;
        private double swerveHeadingKd;
        private double swerveHeadingKf;

        public Builder setWheelbaseLengthInMeters(double wheelbaseLengthInMeters) {
            this.wheelbaseLengthInMeters = wheelbaseLengthInMeters;
            return this;
        }

        public Builder setWheelbaseWidthInMeters(double wheelbaseWidthInMeters) {
            this.wheelbaseWidthInMeters = wheelbaseWidthInMeters;
            return this;
        }

        public Builder setMaxSpeedInMetersPerSecond(double maxSpeedInMetersPerSecond) {
            this.maxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
            return this;
        }

        public Builder setMaxSpeedInRadiansPerSecondLimit(double maxSpeedInRadiansPerSecondLimit) {
            this.maxSpeedInRadiansPerSecondLimit = maxSpeedInRadiansPerSecondLimit;
            return this;
        }

        public Builder setSwerveHeadingKp(double swerveHeadingKp) {
            this.swerveHeadingKp = swerveHeadingKp;
            return this;
        }

        public Builder setSwerveHeadingKi(double swerveHeadingKi) {
            this.swerveHeadingKi = swerveHeadingKi;
            return this;
        }

        public Builder setSwerveHeadingKd(double swerveHeadingKd) {
            this.swerveHeadingKd = swerveHeadingKd;
            return this;
        }

        public Builder setSwerveHeadingKf(double swerveHeadingKf) {
            this.swerveHeadingKf = swerveHeadingKf;
            return this;
        }

        public SwerveConfiguration build() {
            return new SwerveConfiguration(wheelbaseLengthInMeters, wheelbaseWidthInMeters, maxSpeedInMetersPerSecond,
                    maxSpeedInRadiansPerSecondLimit, swerveHeadingKp, swerveHeadingKi, swerveHeadingKd,
                    swerveHeadingKf);
        }
    }
}
