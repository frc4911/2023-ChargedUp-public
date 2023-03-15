package com.cyberknights4911.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.geometry.Pose2d;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String ROBOT_NAME_2023 = "WHAM!";

    public static final String CANIVORE_NAME = "CANivore";

    //2023 Physical Robot Dimensions
    public static final double ROBOT_WHEEL_BASE = 22.75;
    public static final double ROBOT_WHEEL_LENGTH = 22.75;
    public static final double ROBOT_WHEEL_BASE_METERS = Units.inchesToMeters(ROBOT_WHEEL_BASE);
    public static final double ROBOT_WHEEL_LENGTH_METERS = Units.inchesToMeters(ROBOT_WHEEL_LENGTH);
    public static final double ROBOT_HALF_WIDTH = ROBOT_WHEEL_BASE / 2.0;
    public static final double ROBOT_HALF_LENGTH = ROBOT_WHEEL_LENGTH / 2.0;

    // Path following constants
    public static final double PATH_LOOKAHEAD_TIME = 0.25; // seconds to look ahead along the path for steering 0.4
    public static final double PATH_MINI_LOOKAHEAD_DISTANCE = Units.inchesToMeters(6.0); // inches 24.0 (we've been
                                                                                      // using 3.0)
     // Swerve Heading Controller
     public static final double SWERVE_HEADING_CONTROLLER_ERROR_TOLERANCE = 1.0; // degrees


    // These settings are for an inverted Mk4 L2. The steering reduction is
    // different:
    // (14.0/50.0) verses (15.0 / 32.0) on a standard Mk4_L2.
    public static final double MK4_L1_IWHEEL_DIAMETER = 0.10033;
    public static final double MK4_L1_IDRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final boolean MK4_L1_IDRIVE_INVERTED = true;
    public static final double MK4_L1_ISTEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean MK4_L1_ISTEER_INVERTED = true;

    public static final double MK4_L2I_WHEEL_DIAMETER = 0.10033;
    public static final double MK4_L2I_DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean MK4_L2I_DRIVE_INVERTED = true;
    public static final double MK4_L2I_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean MK4_L2_IS_STEER_INVERTED = true;

    public static final double DRIVE_DEADBAND = 0.1;

    // New Swerve requires SI units
    // NOTE: All Robot specific configuration for Junior, DeadEye, and Robot2022 can
    // be found in config folder.
    // Robot specific Swerve configuration is done in SwerveConfiguration
    // Module specific configuration is done in SwerveDriveConstants

    //Robot Starting Pose
    public static final Pose2d ROBOT_STARTING_POSE = new Pose2d();

    // LED/Canifier
    public static final int LONG_CAN_TIMEOUTS_MS = 100; // use for constructors

    // Controller port
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    //auto balance
    public static final boolean PITCH_INVERSION = false;
    public static final boolean ROLL_INVERSION = false;

    public static final double MAX_SPEED = 1;

    // pidIdx for BaseMotorController indicating primary closed-loop
    public static final int PRIMARY_PID = 0;
    
    // remoteOrdinal for BaseMotorController indicating remote sensor 0
    public static final int REMOTE_SENSOR_ZERO = 0;

    // period frame to use for primary PID[0]
    public static final int PRIMARY_PID_PERIOD = 10;
    // period frame to use for motion magic
    public static final int MOTION_MAGIC_PERIOD = 10;
    // period frame to use for selected sensor on primary PID[0]
    public static final int PRIMARY_PID0_PERIOD = 5;

    public static class Arm {
        private Arm() {}
        
        // PID values
        public static final double SHOULDER_P = 0.03;//0.025;
        public static final double SHOULDER_I = 0.0;
        public static final double SHOULDER_D = 0.0;
        public static final double WRIST_P = 0.004;
        public static final double WRIST_I = 0.0;
        public static final double WRIST_D = 0.0;
        public static final double WRIST_F = 0.0399609375;
        public static final double SHOULDER_TOLERANCE = 1000.0;
        public static final double WRIST_TOLERANCE = 1000.0;
        // Arm feed forward
        public static final double SHOULDER_S = 0;
        public static final double SHOULDER_V = 0;
        public static final double SHOULDER_G = 0.02737;
        public static final double WRIST_S = 0;
        public static final double WRIST_V = 0;
        public static final double WRIST_G = 0;

        // Constraints
        public static final double SHOULDER_NEUTRAL_DEADBAND = 0.001;
        public static final double SHOULDER_VELOCITY = 250;
        public static final double SHOULDER_ACCELERATION = 100;
        public static final double SHOULDER_PEAK_OUTPUT = 0.40;
        public static final int SHOULDER_INTEGRAL_ZONE = 100;

        public static final double WRIST_NEUTRAL_DEADBAND = 0.001;
        public static final double WRIST_VELOCITY = 600;
        public static final double WRIST_ACCELERATION = 240;
        public static final double WRIST_PEAK_OUTPUT = 0.75;
        public static final int WRIST_INTEGRAL_ZONE = 100;
        
        // public static final double SHOULDER_VELOCITY_FLIP = 200;
        // public static final double SHOULDER_ACCELERATION_FLIP = 200;
        // public static final double WRIST_VELOCITY_FLIP = 250;
        // public static final double WRIST_ACCELERATION_FLIP = 350;
        
        public static final double PROFILE_ARM_SPEED_STOPPED = 0.0;
        public static final double PROFILE_ARM_SPEED_FORWARD = -50.0;
        public static final double PROFILE_ARM_SPEED_BACKWARD = 50.0;
        
        public static final double SHOULDER_CANCODER_OFFSET = 360 - 233.9;
        public static final double WRIST_CANCODER_OFFSET = 360 - 237.0;

        private static final boolean IS_MOTION_MAGIC = true;
        public static final double WRIST_VELOCITY_MOTION_MAGIC = 600;
        public static final double WRIST_ACCELERATION_MOTION_MAGIC = 240;

        public static final double SHOULDER_VELOCITY_MOTION_MAGIC = 600; //We should increase this a lot currently stow-L3 is 4 seconds needs to be closer to 2
        public static final double SHOULDER_ACCELERATION_MOTION_MAGIC = 600;
        
        public static final SupplyCurrentLimitConfiguration WRIST_SUPPLY_LIMIT =
            new SupplyCurrentLimitConfiguration(true, 30.0, 0, 0);
        public static final StatorCurrentLimitConfiguration WRIST_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 30.0, 0, 0);
        public static final SupplyCurrentLimitConfiguration SHOULDER_SUPPLY_LIMIT =
            new SupplyCurrentLimitConfiguration(true, 30.0, 0, 0);
        public static final StatorCurrentLimitConfiguration SHOULDER_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 30.0, 0, 0);

        public static final FilterConfiguration SHOULDER_FILTER_CONFIG =
            new FilterConfiguration() {
                {
                    remoteSensorDeviceID = Ports.Arm.SHOULDER_CANCODER;
                    remoteSensorSource = RemoteSensorSource.CANCoder;
                }
            };
        public static final FilterConfiguration WRIST_FILTER_CONFIG =
            new FilterConfiguration() {
                {
                    remoteSensorDeviceID = Ports.Arm.WRIST_CANCODER;
                    remoteSensorSource = RemoteSensorSource.CANCoder;
                }
            };
        public static final SlotConfiguration SHOULDER_SLOT_CONFIG =
            new SlotConfiguration() {
                {
                    kP = 0.25;
                    kI = 0.0;
                    kD = 0.0;
                    kF = 0.0;
                    integralZone = SHOULDER_INTEGRAL_ZONE;
                    closedLoopPeakOutput = SHOULDER_PEAK_OUTPUT;
                }
            };
        public static final SlotConfiguration WRIST_SLOT_CONFIG =
            new SlotConfiguration() {
                {
                    kP = 0.25;
                    kI = 0.0;
                    kD = 0.0;
                    kF = 0.0;
                    integralZone = WRIST_INTEGRAL_ZONE;
                    closedLoopPeakOutput = WRIST_PEAK_OUTPUT;
                }
            };

        public static boolean isMotionMagic() {
            return Preferences.getBoolean("Arm Motion Magic enabled", IS_MOTION_MAGIC);
        }
    }

}
