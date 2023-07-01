package com.cyberknights4911.robot.model.wham;

import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.cyberknights4911.robot.constants.BooleanPreference;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.DoublePreference;
import com.cyberknights4911.robot.constants.IntPreference;
import com.cyberknights4911.robot.subsystems.drive.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class WhamConstants {
    private WhamConstants() {}

    public static final String CANIVORE_NAME = "CANivore";

    // Physical Robot Dimensions
    public static final double ROBOT_WHEEL_BASE = 22.75;
    public static final double ROBOT_WHEEL_LENGTH = 22.75;
    public static final double ROBOT_WHEEL_BASE_METERS = Units.inchesToMeters(ROBOT_WHEEL_BASE);
    public static final double ROBOT_WHEEL_LENGTH_METERS = Units.inchesToMeters(ROBOT_WHEEL_LENGTH);
    public static final double ROBOT_HALF_WIDTH = ROBOT_WHEEL_BASE / 2.0;
    public static final double ROBOT_HALF_LENGTH = ROBOT_WHEEL_LENGTH / 2.0;

    public static final int LONG_CAN_TIMEOUTS_MS = 100; // use for constructors

    // Controller port
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

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

    public static final class Drive {
        private Drive() {}

        private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 20.65;
        private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 159.50;
        private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 194.00;
        private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 104.67;

        private static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
            CotsFalconSwerveConstants.SDSMK4i(
                CotsFalconSwerveConstants.DriveGearRatios.SDSMK4i_L1
            );

        public static final SwerveModuleConstants FRONT_LEFT = 
            new SwerveModuleConstants(
                WhamPorts.Drive.FRONT_LEFT_DRIVE,
                WhamPorts.Drive.FRONT_LEFT_STEER,
                WhamPorts.Drive.FRONT_LEFT_CANCODER,
                Rotation2d.fromDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants FRONT_RIGHT = 
            new SwerveModuleConstants(
                WhamPorts.Drive.FRONT_RIGHT_DRIVE,
                WhamPorts.Drive.FRONT_RIGHT_STEER,
                WhamPorts.Drive.FRONT_RIGHT_CANCODER,
                Rotation2d.fromDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants BACK_LEFT = 
            new SwerveModuleConstants(
                WhamPorts.Drive.BACK_LEFT_DRIVE,
                WhamPorts.Drive.BACK_LEFT_STEER,
                WhamPorts.Drive.BACK_LEFT_CANCODER,
                Rotation2d.fromDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants BACK_RIGHT = 
            new SwerveModuleConstants(
                WhamPorts.Drive.BACK_RIGHT_DRIVE,
                WhamPorts.Drive.BACK_RIGHT_STEER,
                WhamPorts.Drive.BACK_RIGHT_CANCODER,
                Rotation2d.fromDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES));
    }

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
                    remoteSensorDeviceID = WhamPorts.Arm.SHOULDER_CANCODER;
                    remoteSensorSource = RemoteSensorSource.CANCoder;
                }
            };
        public static final FilterConfiguration WRIST_FILTER_CONFIG =
            new FilterConfiguration() {
                {
                    remoteSensorDeviceID = WhamPorts.Arm.WRIST_CANCODER;
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

    public static final class Slurpp {
        private Slurpp() {}

    public static final SupplyCurrentLimitConfiguration SLURPP_SUPPLY_LIMIT =
        new SupplyCurrentLimitConfiguration(true, 35.0, 0, 0);
    public static final StatorCurrentLimitConfiguration SLURPP_STATOR_LIMIT =
        new StatorCurrentLimitConfiguration(true, 35.0, 0, 0);
    }
    
}
