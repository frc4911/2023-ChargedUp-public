package com.cyberknights4911.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String ROBOT_NAME = "QuickSlurpp";

    public static final String ROBOT_NAME_2022 = "Robot2022";

    public static final String CANIVORE_NAME = "CANivore";

    //2022 Physical Robot Dimensions (including bumpers)
    public static final double ROBOT_WIDTH = 33;
    public static final double ROBOT_LENGTH = 34.5;
    public static final double ROBOT_WIDTH_METERS = 0.8382;
    public static final double ROBOT_LENGTH_METERS = 0.8763;
    public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0;
    public static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0;

    public static final double ROBOT_WIDTH_METERS_2023 = 0.8763;
    public static final double ROBOT_LENGTH_METERS_2023 = 0.8763;

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

    public static final double DRIVE_DEADBAND = 0.05;

    // New Swerve requires SI units
    // NOTE: All Robot specific configuration for Junior, DeadEye, and Robot2022 can
    // be found in config folder.
    // Robot specific Swerve configuration is done in SwerveConfiguration
    // Module specific configuration is done in SwerveDriveConstants

    //Robot Starting Pose
    public static final Pose2d ROBOT_STARTING_POSE = new Pose2d();

    // END NEW SWERVE

    // LED/Canifier
    public static final int LONG_CAN_TIMEOUTS_MS = 100; // use for constructors

    // Controller port
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    // Claw Talon ID
    public static final int CLAW_MOTOR_ID = 1;

    //auto balance
    public static final boolean PITCH_INVERSION = false;
    public static final boolean ROLL_INVERSION = false;

    public static final double MAX_SPEED = 1;

}
