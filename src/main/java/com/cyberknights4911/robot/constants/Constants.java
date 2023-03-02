package com.cyberknights4911.robot.constants;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.control.PidGains;
import edu.wpi.first.math.geometry.Pose2d;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String ROBOT_NAME_2023 = "Robot2023";

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

    // Arm
    // PID values
    // public static final double SHOULDER_P = 0.04;
    // public static final double SHOULDER_I = 0.0;
    // public static final double SHOULDER_D = 0.00034;
    // public static final double WRIST_P = 0.04;
    // public static final double WRIST_I = 0.0;
    // public static final double WRIST_D = 0.002;
    public static final double SHOULDER_P = 0.03;
    public static final double SHOULDER_I = 0.0;
    public static final double SHOULDER_D = 0.0;
    public static final double WRIST_P = 0.02;
    public static final double WRIST_I = 0.0;
    public static final double WRIST_D = 0.0;//0.0015;
    public static final double SHOULDER_TOLERANCE = 1.0;
    public static final double WRIST_TOLERANCE = 1.0;

    // Arm feed forward
    public static final double SHOULDER_S = 0;
    public static final double SHOULDER_V = 0;
    public static final double SHOULDER_G = 0;
    public static final double WRIST_S = 0;
    public static final double WRIST_V = 0;
    public static final double WRIST_G = 0;

    // Constraints
    // public static final double SHOULDER_VELOCITY = 25;
    // public static final double SHOULDER_ACCELERATION = 37.5;
    // public static final double WRIST_VELOCITY = 30;
    // public static final double WRIST_ACCELERATION = 45;
    public static final double SHOULDER_VELOCITY = 200;
    public static final double SHOULDER_ACCELERATION = 150;
    public static final double WRIST_VELOCITY = 100;
    public static final double WRIST_ACCELERATION = 75;

}
