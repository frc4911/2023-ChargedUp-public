package com.cyberknights4911.robot.constants;

import edu.wpi.first.math.util.Units;
import libraries.cheesylib.geometry.Pose2d;
//import frc.robot.limelight.CameraResolution;
//import frc.robot.limelight.PipelineConfiguration;
//import libraries.cheesylib.geometry.Pose2d;
//import libraries.cheesylib.geometry.Rotation2d;
//import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.vision.GoalTrackerConfig;

public class Constants {
    /* All distance measurements are in inches, unless otherwise noted. */

    public static final String ROBOT_NAME = "QuickSuck";

    public static final String ROBOT_NAME_2022 = "Robot2022";

    public static final String CANIVORE_NAME = "CANivore";

    public static final double LOOPER_DT = 0.02;

    public static final double EPSILON = 0.0001;

    public static final boolean IS_USING_COMPBOT = true;
    public static final boolean IS_USING_TRACTION_WHEELS = true;

    public static final boolean DEBUGGING_OUTPUT = true;

    // Physical Robot Dimensions (including bumpers)
    public static final double ROBOT_WIDTH = 28.0;
    public static final double ROBOT_LENGTH = 28.0;
    public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0;
    public static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0;
    public static final double ROBOT_PROBE_EXTRUSION = 4.0;

    public static final double BALL_RADIUS = 6.5;


    // Path following constants
    public static final double PATH_LOOKAHEAD_TIME = 0.25; // seconds to look ahead along the path for steering 0.4
    public static final double PATH_MINI_LOOKAHEAD_DISTANCE = Units.inchesToMeters(6.0); // inches 24.0 (we've been
                                                                                      // using 3.0)
    

    // These settings are for an inverted Mk4 L2. The steering reduction is
    // different:
    // (14.0/50.0) verses (15.0 / 32.0) on a standard Mk4_L2.
    public static final double MK4_L1_IWHEEL_DIAMETER = 0.10033;
    public static final double MK4_L1_IDRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final boolean MK4_L1_IDRIVE_INVERTED = true;
    public static final double MK4_L1_ISTEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean MK4_L1_ISTEER_INVERTED = true;

    public static final double DRIVE_DEADBAND = 0.05;

    // New Swerve requires SI units
    // NOTE: All Robot specific configuration for Junior, DeadEye, and Robot2022 can
    // be found in config folder.
    // Robot specific Swerve configuration is done in SwerveConfiguration
    // Module specific configuration is done in SwerveDriveConstants

    //Robot Starting Pose
    public static final Pose2d ROBOT_STARTING_POSE = new Pose2d();

    // Swerve Heading Controller
    public static final double SWERVE_HEADING_CONTROLLER_ERROR_TOLERANCE = 1.0; // degrees

    // END NEW SWERVE

    // LIMELIGHT
    // TODO:  Create Robot specific configurations
    // Goal Tracker
    public static final boolean USE_TOP_CORNERS = true;

    public static final double IMAGE_CAPUTRE_LATENCY = 11.0 / 1000.0; // seconds

    public static final double MAX_TRACKER_DISTANCE = Units.feetToMeters(16.0);
    public static final double MAX_GOAL_TRACK_AGE = 1.0; //2.5;
    public static final double MAX_GOAL_TRACK_SMOOTHING_TIME = 0.5;
    public static final double CAMERA_FRAME_RATE = 90.0; // fps

    public static GoalTrackerConfig GOAL_TRACKER_CONFIG = new GoalTrackerConfig(
            MAX_TRACKER_DISTANCE, MAX_GOAL_TRACK_AGE, MAX_GOAL_TRACK_SMOOTHING_TIME, CAMERA_FRAME_RATE);

    public static final double TRACK_STABILITY_WEIGHT = 0.0;
    public static final double TRACK_AGE_WEIGHT = 10.0;
    public static final double TRACK_SWITCHING_WEIGHT = 100.0;

    // LED/Canifier
    public static final int CANIFER_ID = 0;
    public static final int CAN_TIMEOUTS_MS = 10; // use for on the fly updates
    public static final int LONG_CAN_TIMEOUTS_MS = 100; // use for constructors

    // Controller port
    public static final int DRIVER_CONTROLLER_PORT = 0;
}