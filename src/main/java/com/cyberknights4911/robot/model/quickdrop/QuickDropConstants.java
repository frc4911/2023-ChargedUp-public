package com.cyberknights4911.robot.model.quickdrop;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.cyberknights4911.robot.drive.swerve.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.drive.swerve.SwerveDriveConstants;
import com.cyberknights4911.robot.drive.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class QuickDropConstants {

    public static final String CANIVORE_NAME = "CANivore";
    public static final int LONG_CAN_TIMEOUTS_MS = 100;

    private QuickDropConstants() {}

    public static final class Drive {
        private Drive() {}

        private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 330;
        private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 182.5;
        private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 262.7;
        private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 64.5;

        private static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
        private static final double WHEEL_BASE = Units.inchesToMeters(23.75);

        private static final boolean INVERT_GYRO = false;

        private static final double STICK_DEADBAND = 0.1;

        /* Meters per Second */
        private static final double MAX_SPEED = 4.5;
        /* Radians per Second */
        private static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Swerve Current Limiting */
        private static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        private static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        private static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        private static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        private static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        private static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        private static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        private static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        private static final double OPEN_LOOP_RAMP = 0.25;
        private static final double CLOSED_LOOP_RAMP = 0.0;
        
        /* Drive Motor PID Values */
        private static final double DRIVE_KP = 0.05;
        private static final double DRIVE_KI = 0.0;
        private static final double DRIVE_KD = 0.0;
        private static final double DRIVE_KF = 0.0;

        /* 
         * Drive Motor Characterization Values.
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        private static final double DRIVE_KS = (0.32 / 12.0);
        private static final double DRIVE_KV = (1.51 / 12.0);
        private static final double DRIVE_KA = (0.27 / 12.0);

        /* Neutral Modes */
        private static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        private static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        public static final SwerveDriveConstants SWERVE_DRIVE_CONSTANTS =
            SwerveDriveConstants.builder()
                .setWheelBase(WHEEL_BASE)
                .setTrackWidth(TRACK_WIDTH)
                .setInvertGyro(INVERT_GYRO)
                .setGyroId(QuickDropPorts.Drive.PIGEON)
                .setMaxSpeed(MAX_SPEED)
                .setMaxAngularVelocity(MAX_ANGULAR_VELOCITY)
                .setStickDeadband(STICK_DEADBAND)
                .setAngleContinuousCurrentLimit(ANGLE_CONTINUOUS_CURRENT_LIMIT)
                .setAnglePeakCurrentLimit(ANGLE_PEAK_CURRENT_LIMIT)
                .setAnglePeakCurrentDuration(ANGLE_PEAK_CURRENT_DURATION)
                .setAngleEnableCurrentLimit(ANGLE_ENABLE_CURRENT_LIMIT)
                .setDriveContinuousCurrentLimit(DRIVE_CONTINUOUS_CURRENT_LIMIT)
                .setDrivePeakCurrentLimit(DRIVE_PEAK_CURRENT_LIMIT)
                .setDrivePeakCurrentDuration(DRIVE_PEAK_CURRENT_DURATION)
                .setDriveEnableCurrentLimit(DRIVE_ENABLE_CURRENT_LIMIT)
                .setOpenloopRamp(OPEN_LOOP_RAMP)
                .setClosedloopRamp(CLOSED_LOOP_RAMP)
                .setDriveProportionalGain(DRIVE_KP)
                .setDriveIntegralGain(DRIVE_KI)
                .setDriveDerivativeGain(DRIVE_KD)
                .setDriveFeedForwardGain(DRIVE_KF)
                .setDriveStaticGain(DRIVE_KS)
                .setDriveVelocityGain(DRIVE_KV)
                .setDriveAccelerationGain(DRIVE_KA)
                .setAngleNeutralMode(ANGLE_NEUTRAL_MODE)
                .setDriveNeutralMode(DRIVE_NEUTRAL_MODE)
                .build();

        public static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
            CotsFalconSwerveConstants.SDSMK4i(
                CotsFalconSwerveConstants.DriveGearRatios.SDSMK4i_L1
            );

        public static final SwerveModuleConstants FRONT_LEFT = 
            SwerveModuleConstants.builder()
                .setDriveMotorId(QuickDropPorts.Drive.FRONT_LEFT_DRIVE)
                .setAngleMotorId(QuickDropPorts.Drive.FRONT_LEFT_STEER)
                .setCanCoderId(QuickDropPorts.Drive.FRONT_LEFT_CANCODER)
                .setAngleOffset(Rotation2d.fromDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES))
                .build();

        public static final SwerveModuleConstants FRONT_RIGHT = 
            SwerveModuleConstants.builder()
                .setDriveMotorId(QuickDropPorts.Drive.FRONT_RIGHT_DRIVE)
                .setAngleMotorId(QuickDropPorts.Drive.FRONT_RIGHT_STEER)
                .setCanCoderId(QuickDropPorts.Drive.FRONT_RIGHT_CANCODER)
                .setAngleOffset(Rotation2d.fromDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES))
                .build();

        public static final SwerveModuleConstants BACK_LEFT = 
            SwerveModuleConstants.builder()
                .setDriveMotorId(QuickDropPorts.Drive.BACK_LEFT_DRIVE)
                .setAngleMotorId(QuickDropPorts.Drive.BACK_LEFT_STEER)
                .setCanCoderId(QuickDropPorts.Drive.BACK_LEFT_CANCODER)
                .setAngleOffset(Rotation2d.fromDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES))
                .build();

        public static final SwerveModuleConstants BACK_RIGHT = 
            SwerveModuleConstants.builder()
                .setDriveMotorId(QuickDropPorts.Drive.BACK_RIGHT_DRIVE)
                .setAngleMotorId(QuickDropPorts.Drive.BACK_RIGHT_STEER)
                .setCanCoderId(QuickDropPorts.Drive.BACK_RIGHT_CANCODER)
                .setAngleOffset(Rotation2d.fromDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES))
                .build();
    }
    
    public static class Shooter {
        private Shooter() {}
        
        public static final double MIN_SHOOT_DISTANCE = 0; // Fender shot is 0
        public static final double MAX_SHOOT_DISTANCE = 144; // inches

        public static final double FLYWHEEL_P = 0.175;
        public static final double FLYWHEEL_I = 0.0;//001;
        public static final double FLYWHEEL_D = 2.0;
        public static final double FLYWHEEL_F = 0.05;
        public static final double FLYWHEEL_CLOSED_RAMP = 1;
        public static final double FLYWHEEL_CLOSED_ERROR = 25.0;
        public static final double FLYWHEEL_INTEGRAL_ZONE = 700;

        public static final double POSITION_AT_FENDER = 8000;

        public static final double HOOD_P = 0.7;
        public static final double HOOD_I = 0.04;
        public static final double HOOD_D = 10.0;
        public static final double HOOD_F = 0.0;
        public static final double HOOD_CLOSED_RAMP = 0.0;
        public static final double HOOD_CLOSED_ERROR = 15.0;
        public static final double HOOD_INTEGRAL_ZONE = 200;
        public static final int HOOD_MOTION_CRUISE_VELOCITY = 10000;
        public static final int HOOD_MOTION_ACCELERATION = 5400;
        public static final int HOOD_MOTION_S_CURVE_STRENGTH = 0;

        public static final StatorCurrentLimitConfiguration FLYWHEEL_LEFT_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 40.0, 40.0, 0);
        public static final StatorCurrentLimitConfiguration FLYWHEEL_RIGHT_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 40.0, 40.0, 0);
        public static final StatorCurrentLimitConfiguration HOOD_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 30.0, 30.0, 0);
        
    }
}
