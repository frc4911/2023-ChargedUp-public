package frc.robot.constants;

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

    public static final String kCanivoreName = "CANivore";

    public static final double kLooperDt = 0.02;

    public static final double kEpsilon = 0.0001;

    public static final boolean kIsUsingCompBot = true;
    public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kDebuggingOutput = true;

    // Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 28.0;
    public static final double kRobotLength = 28.0;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    public static final double kRobotProbeExtrusion = 4.0;

    public static final double kBallRadius = 6.5;


    // Path following constants
    public static final double kPathLookaheadTime = 0.25; // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = Units.inchesToMeters(6.0); // inches 24.0 (we've been
                                                                                      // using 3.0)
    

    // These settings are for an inverted Mk4 L2. The steering reduction is
    // different:
    // (14.0/50.0) verses (15.0 / 32.0) on a standard Mk4_L2.
    public static final double kMK4_L2iWheelDiameter = 0.10033;
    public static final double kMK4_L2iDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean kMK4_L2iDriveInverted = true;
    public static final double kMK4_L2iSteerReduction = (14.0 / 50.0) * (10.0 / 60.0);
    public static final boolean kMK4_L2iSteerInverted = true;

    public static final double kDriveDeadband = 0.05;

    // New Swerve requires SI units
    // NOTE: All Robot specific configuration for Junior, DeadEye, and Robot2022 can
    // be found in config folder.
    // Robot specific Swerve configuration is done in SwerveConfiguration
    // Module specific configuration is done in SwerveDriveConstants

    //Robot Starting Pose
    public static final Pose2d kRobotStartingPose = new Pose2d();

    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.0; // degrees

    // END NEW SWERVE

    // LIMELIGHT
    // TODO:  Create Robot specific configurations
    // Goal Tracker
    public static final boolean kUseTopCorners = true;

    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = Units.feetToMeters(16.0);
    public static final double kMaxGoalTrackAge = 1.0; //2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0; // fps

    public static GoalTrackerConfig kGoalTrackerConfig = new GoalTrackerConfig(
            kMaxTrackerDistance, kMaxGoalTrackAge, kMaxGoalTrackSmoothingTime, kCameraFrameRate);

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    // LED/Canifier
    public static final int kCanifierId = 0;
    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Controller port
    public static final int kDriverControllerPort = 0;
}
