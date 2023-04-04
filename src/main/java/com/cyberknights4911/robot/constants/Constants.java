package com.cyberknights4911.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.util.Units;

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

    /**
     * Used to control whether to log to NetworkTables. This can be resource intensive so it should
     * be disabled for competitions.
     */
    public static final boolean ENABLE_NT_LOGGING = true;

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
    public static final double MK4_L1I_WHEEL_DIAMETER = 0.10033;
    public static final double MK4_L1I_DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final boolean MK4_L1I_DRIVE_INVERTED = true;
    public static final double MK4_L1I_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean MK4_L1I_STEER_INVERTED = true;

    public static final double MK4_L2I_WHEEL_DIAMETER = 0.10033;
    public static final double MK4_L2I_DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean MK4_L2I_DRIVE_INVERTED = true;
    public static final double MK4_L2I_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean MK4_L2_IS_STEER_INVERTED = true;

    public static final double DRIVE_DEADBAND = 0.05;

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
    public static final boolean YAW_INVERSION = false;

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

    public static final SupplyCurrentLimitConfiguration SLURPP_SUPPLY_LIMIT =
        new SupplyCurrentLimitConfiguration(true, 35.0, 0, 0);
    public static final StatorCurrentLimitConfiguration SLURPP_STATOR_LIMIT =
        new StatorCurrentLimitConfiguration(true, 35.0, 0, 0);

    public static class Arm {
        private Arm() {}

        public static final boolean IS_TUNING_ENABLED = false;

        public static BooleanPreference SHOULD_USE_GRAVITY_FEED_FORWARD =
            new BooleanPreference("Should use shoulder gravity feed forward", true);
            
        public static final DoublePreference WRIST_TUCKED_ANGLE_BACK_TO_FRONT = 
            new DoublePreference("WRIST_TUCKED_ANGLE_BACK_TO_FRONT", 315);
        public static final DoublePreference WRIST_TUCKED_ANGLE_FRONT_TO_BACK = 
            new DoublePreference("WRIST_TUCKED_ANGLE_FRONT_TO_BACK", 70);
        public static final DoublePreference SHOULDER_SAFE_ANGLE_FRONT = 
            new DoublePreference("SHOULDER_SAFE_ANGLE_FRONT", 142);
        public static final DoublePreference SHOULDER_SAFE_ANGLE_BACK_TOP = 
            new DoublePreference("SHOULDER_SAFE_ANGLE_BACK_TOP", 200);
        public static final DoublePreference SHOULDER_SAFE_ANGLE_BACK_MIDDLE = 
            new DoublePreference("SHOULDER_SAFE_ANGLE_BACK_MIDDLE", 245);
        public static final DoublePreference SHOULDER_SAFE_ANGLE_BACK_BOTTOM = 
            new DoublePreference("SHOULDER_SAFE_ANGLE_BACK_BOTTOM", 295);

        public static final DoublePreference STOWED_WRIST =
            new DoublePreference("STOWED_WRIST", 325);
        public static final DoublePreference STOWED_SHOULDER =
            new DoublePreference("STOWED_SHOULDER", 38);
        public static final DoublePreference COLLECT_SINGLE_SUBSTATION_FRONT_WRIST =
            new DoublePreference("COLLECT_SINGLE_SUBSTATION_FRONT_WRIST", 99);
        public static final DoublePreference COLLECT_SINGLE_SUBSTATION_FRONT_SHOULDER =
            new DoublePreference("COLLECT_SINGLE_SUBSTATION_FRONT_SHOULDER", 103);
        public static final DoublePreference COLLECT_SUBSTATION_BACK_WRIST =
            new DoublePreference("COLLECT_SUBSTATION_BACK_WRIST", 155);
        public static final DoublePreference COLLECT_SUBSTATION_BACK_SHOULDER =
            new DoublePreference("COLLECT_SUBSTATION_BACK_SHOULDER", 258);
        public static final DoublePreference COLLECT_SUBSTATION_FRONT_WRIST =
            new DoublePreference("COLLECT_SUBSTATION_FRONT_WRIST", 223);
        public static final DoublePreference COLLECT_SUBSTATION_FRONT_SHOULDER =
            new DoublePreference("COLLECT_SUBSTATION_FRONT_SHOULDER", 101);
        public static final DoublePreference COLLECT_FLOOR_FRONT_CONE_WRIST =
            new DoublePreference("COLLECT_FLOOR_FRONT_CONE_WRIST", 265);
        public static final DoublePreference COLLECT_FLOOR_FRONT_CONE_SHOULDER =
            new DoublePreference("COLLECT_FLOOR_FRONT_CONE_SHOULDER", 38);
        public static final DoublePreference COLLECT_FLOOR_FRONT_CUBE_WRIST =
            new DoublePreference("COLLECT_FLOOR_FRONT_CUBE_WRIST", 247);
        public static final DoublePreference COLLECT_FLOOR_FRONT_CUBE_SHOULDER =
            new DoublePreference("COLLECT_FLOOR_FRONT_CUBE_SHOULDER", 38);
        public static final DoublePreference COLLECT_FLOOR_BACK_CUBE_WRIST =
            new DoublePreference("COLLECT_FLOOR_BACK_CUBE_WRIST", 117);
        public static final DoublePreference COLLECT_FLOOR_BACK_CUBE_SHOULDER =
            new DoublePreference("COLLECT_FLOOR_BACK_CUBE_SHOULDER", 320);
        public static final DoublePreference COLLECT_FLOOR_BACK_CONE_WRIST =
            new DoublePreference("COLLECT_FLOOR_BACK_CONE_WRIST", 105);
        public static final DoublePreference COLLECT_FLOOR_BACK_CONE_SHOULDER =
            new DoublePreference("COLLECT_FLOOR_BACK_CONE_SHOULDER", 320);
        public static final DoublePreference SCORE_L3_WRIST =
            new DoublePreference("SCORE_L3_WRIST", 195);
        public static final DoublePreference SCORE_L3_SHOULDER =
            new DoublePreference("SCORE_L3_SHOULDER", 235);
        public static final DoublePreference SCORE_L2_WRIST =
            new DoublePreference("SCORE_L2_WRIST", 211);
        public static final DoublePreference SCORE_L2_SHOULDER =
            new DoublePreference("SCORE_L2_SHOULDER", 93);
        
        
        // PID values
        public static final DoublePreference SHOULDER_P = 
            new DoublePreference("SHOULDER_P", .7);
        public static final DoublePreference SHOULDER_I = 
            new DoublePreference("SHOULDER_I", 0.0);
        public static final DoublePreference SHOULDER_D = 
            new DoublePreference("SHOULDER_D", 0.0);
            public static final DoublePreference SHOULDER_F = 
                new DoublePreference("SHOULDER_F", 1);
        public static final DoublePreference SHOULDER_G = 
            new DoublePreference("SHOULDER_G", 0.02737);

        public static final DoublePreference WRIST_P = 
            new DoublePreference("WRIST_P", 0.4);
        public static final DoublePreference WRIST_I = 
            new DoublePreference("WRIST_I", 0.0);
        public static final DoublePreference WRIST_D = 
            new DoublePreference("WRIST_D", 0.0);
        public static final DoublePreference WRIST_F = 
            new DoublePreference("WRIST_F", 0.5);

        // Constraints
        public static final DoublePreference SHOULDER_NEUTRAL_DEADBAND =
            new DoublePreference("SHOULDER_NEUTRAL_DEADBAND", 0.001);
        public static final DoublePreference SHOULDER_PEAK_OUTPUT =
            new DoublePreference("SHOULDER_PEAK_OUTPUT", 0.5);
        public static final IntPreference SHOULDER_INTEGRAL_ZONE =
            new IntPreference("SHOULDER_INTEGRAL_ZONE", 100);

        public static final DoublePreference WRIST_NEUTRAL_DEADBAND =
            new DoublePreference("WRIST_NEUTRAL_DEADBAND", 0.001);
        public static final DoublePreference WRIST_PEAK_OUTPUT =
            new DoublePreference("WRIST_PEAK_OUTPUT", 1);
        public static final IntPreference WRIST_INTEGRAL_ZONE =
            new IntPreference("WRIST_INTEGRAL_ZONE", 100);
        
        public static final DoublePreference SHOULDER_CANCODER_OFFSET = 
            new DoublePreference("SHOULDER_CANCODER_OFFSET", 67.5);
        public static final DoublePreference WRIST_CANCODER_OFFSET = 
            new DoublePreference("WRIST_CANCODER_OFFSET", 40.5);

        public static final DoublePreference WRIST_VELOCITY_MOTION_MAGIC = 
            new DoublePreference("WRIST_VELOCITY_MOTION_MAGIC", 10000);
        public static final DoublePreference WRIST_ACCELERATION_MOTION_MAGIC = 
            new DoublePreference("WRIST_ACCELERATION_MOTION_MAGIC", 10000);

        public static final DoublePreference SHOULDER_VELOCITY_MOTION_MAGIC = 
            new DoublePreference("SHOULDER_VELOCITY_MOTION_MAGIC", 18000);
        public static final DoublePreference SHOULDER_ACCELERATION_MOTION_MAGIC = 
            new DoublePreference("SHOULDER_ACCELERATION_MOTION_MAGIC", 18000);
        
        public static final SupplyCurrentLimitConfiguration WRIST_SUPPLY_LIMIT =
            new SupplyCurrentLimitConfiguration(true, 40.0, 0, 0);
        public static final StatorCurrentLimitConfiguration WRIST_STATOR_LIMIT =
            new StatorCurrentLimitConfiguration(true, 40.0, 0, 0);
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
                    kP = SHOULDER_P.getValue();
                    kI = SHOULDER_I.getValue();
                    kD = SHOULDER_D.getValue();
                    kF = SHOULDER_F.getValue();
                    integralZone = SHOULDER_INTEGRAL_ZONE.getValue();
                    closedLoopPeakOutput = SHOULDER_PEAK_OUTPUT.getValue();
                }
            };
        public static final SlotConfiguration WRIST_SLOT_CONFIG =
            new SlotConfiguration() {
                {
                    kP = WRIST_P.getValue();
                    kI = WRIST_I.getValue();
                    kD = WRIST_D.getValue();
                    kF = WRIST_F.getValue();
                    integralZone = WRIST_INTEGRAL_ZONE.getValue();
                    closedLoopPeakOutput = WRIST_PEAK_OUTPUT.getValue();
                }
            };
    }
    
    public static class Swerve {
        private Swerve() {}

        public static final double STICK_DEADBAND = 0.1;

        //TODO: This must be tuned to specific robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.75);

        public static final boolean INVERT_GYRO = false;

        /* Meters per Second */
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot
            
        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
        * These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
        */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;
        
        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* 
        * Drive Motor Characterization Values.
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE
        */
        public static final double DRIVE_KS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
    }
}
