package com.cyberknights4911.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.cyberknights4911.robot.commands.DefaultSwerveCommand;
import com.cyberknights4911.robot.config.RobotConfiguration;
import com.cyberknights4911.robot.config.SwerveConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.StickAction;
import com.cyberknights4911.robot.sensors.IMU;

import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import libraries.cyberlib.utils.Util;

public class SwerveSubsystemCurrent implements SwerveSubsystem {

    public enum ControlState {
        NEUTRAL, MANUAL, DISABLED, PATH_FOLLOWING, VISION_AIM
    }

    private ControlState mControlState = ControlState.NEUTRAL;

    public SwerveConfiguration mSwerveConfiguration;
    public RobotConfiguration mRobotConfiguration;

    PeriodicIO mPeriodicIO = new PeriodicIO();
 
   // Module declaration
   private final List<SwerveDriveModule> mModules = new ArrayList<>();
   private final SwerveDriveModule mFrontRight;
   private final SwerveDriveModule mFrontLeft;
   private final SwerveDriveModule mBackLeft;
   private final SwerveDriveModule mBackRight;

   private double lastUpdateTimestamp = 0;
   private int driveMode = 3;
   private boolean inAimingDeadzone;
   private final double kDefaultScaler = 4.0;//2.5; // May need to be increased in Houston
   private double mAimingScaler = kDefaultScaler;

   // Swerve kinematics & odometry
   private final IMU mIMU;
   private boolean mIsBrakeMode;

   // TODO - do we still need this a odometry tracks this already?
   private Rotation2d mGyroOffset = new Rotation2d();

   private final SwerveDriveOdometry mOdometry;
   private final SwerveDriveKinematics mKinematics;

   // Aiming Controller to turn in place when using vision system to aim
//    private SynchronousPIDF mAimingController = new SynchronousPIDF(
//            Constants.kAimingKP, Constants.kAimingKI, Constants.kAimingKD
//    );

//    private static HeadingController mAimingHeaderController = null;

    private double lastAimTimestamp = -1.0;

    // Trajectory following
    private boolean mOverrideTrajectory = false;

    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.0, 0); // 1.5
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0, 0); // 1.5
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2, 0);

    public SwerveSubsystemCurrent() {
        mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.ROBOT_NAME_2022);

        mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();
        mModules.add(mFrontRight = new SwerveDriveModule(mRobotConfiguration.getFrontRightModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mFrontLeft = new SwerveDriveModule(mRobotConfiguration.getFrontLeftModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mBackLeft = new SwerveDriveModule(mRobotConfiguration.getBackLeftModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));
        mModules.add(mBackRight = new SwerveDriveModule(mRobotConfiguration.getBackRightModuleConstants(),
                mSwerveConfiguration.maxSpeedInMetersPerSecond));

        // precaution to ensure misconfiguration modules don't run.
        stopSwerveDriveModules();

        mIMU = IMU.createImu(mRobotConfiguration.getImuType());
        mKinematics = new SwerveDriveKinematics(
            mSwerveConfiguration.moduleLocations.get(0),
            mSwerveConfiguration.moduleLocations.get(1),
            mSwerveConfiguration.moduleLocations.get(2),
            mSwerveConfiguration.moduleLocations.get(3)); 


            var frontRightPosition = mFrontRight.getPosition();
            var frontLeftPosition = mFrontLeft.getPosition();
            var backLeftPosition = mBackLeft.getPosition();
            var backRightPosition = mBackRight.getPosition();



            
        mOdometry = new SwerveDriveOdometry(mKinematics, mIMU.getYaw(),new SwerveModulePosition []{ frontRightPosition, frontLeftPosition, backLeftPosition, backRightPosition} , new Pose2d());
        mPeriodicIO.robotPose = mOdometry.getPoseMeters();

        // rotationPow = SmartDashboard.getNumber("Rotation Power", -1);
        // if(rotationPow == -1) {
        //     SmartDashboard.putNumber("Rotation Power", 0);
        // }
        CommandScheduler.getInstance().registerSubsystem(this);
        convertCancoderToFX();
    }

    @Override
    public void periodic() {
        synchronized (SwerveSubsystemCurrent.this) {
            lastUpdateTimestamp = Timer.getFPGATimestamp();

            readPeriodicInputs();            

            // Update odometry in every loop before any other actions.
            updateOdometry(lastUpdateTimestamp);

            switch (mControlState) {
                case MANUAL:
                    handleManual();
                    break;
                case PATH_FOLLOWING:
                    break;
                case VISION_AIM:
                    handleAiming(lastUpdateTimestamp);
                    break;
                case NEUTRAL:
                case DISABLED:
                default:
                    break;
            }

            writePeriodicOutputs();
        }
    }

    public synchronized void stop() {
        // mControlState = ControlState.NEUTRAL;
        setState(ControlState.NEUTRAL);
        stopSwerveDriveModules();
    }

    private void stopSwerveDriveModules() {
        mModules.forEach((m) -> m.stop());
    }

    public void convertCancoderToFX(){
        mModules.forEach((m) -> m.convertCancoderToFX2());
    }

    public synchronized void zeroSensors() {
        zeroSensors(Constants.ROBOT_STARTING_POSE);
    }

    public void toggleThroughDriveModes() {
        driveMode = ++driveMode % 4;
        System.out.println("Shifting Drive Mode***** to " + driveMode);
        switch (driveMode) {
            case 0:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " SwerveDriveHelper");
                break;
            case 1:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Matt's Algorithm");
                break;
            case 2:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Squared Inputs");
                break;
            case 3:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Raw");
                break;
            default:
                SmartDashboard.putString("Swerve/DriveMode", driveMode + " Unknown");
                break;
        }
    }

    /**
     * Handles MANUAL state which corresponds to joy stick inputs.
     *
     * <p>Using the joy stick values in PeriodicIO, calculate and updates the swerve
     * states. The joy stick values are as percent [-1.0, 1.0]. They need to be
     * converted to SI units before creating the ChassisSpeeds.</p>
     */
    private void handleManual() {
        // Default drivesignal
        HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
            new Translation2d(mPeriodicIO.forward, mPeriodicIO.strafe).times(0.75),
            mPeriodicIO.rotation, 
            mPeriodicIO.field_relative);

        // TODO: Check deadband code in JStick (which reduces range and still returns 0 .s low).
        //  Should always get raw values here and apply deadband code here.
        switch (driveMode) {
            case 0:
                // driveSignal = SwerveDriveHelper.calculate(
                //         mPeriodicIO.forward, mPeriodicIO.strafe, mPeriodicIO.rotation,
                //         mPeriodicIO.low_power, mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller);
                break;
            case 1:
                // Matt's Swerve control
                double driveScalar = 1;
                if (mPeriodicIO.low_power) {
                     driveScalar = 0.25;
                }

                driveSignal = new HolonomicDriveSignal(
                    new Translation2d(mPeriodicIO.forward, mPeriodicIO.strafe).times(driveScalar * 0.75),
                    mPeriodicIO.rotation * driveScalar * 0.8,
                        mPeriodicIO.field_relative);
                break;
            case 2:
                // Inputs squared
                // rotationPow = SmartDashboard.getNumber("Rotation Power", 1);
                driveSignal = new HolonomicDriveSignal(
                        new Translation2d(
                                Math.copySign(Math.pow(mPeriodicIO.forward, 2), mPeriodicIO.forward),
                                Math.copySign(Math.pow(mPeriodicIO.strafe, 2), mPeriodicIO.strafe)),
                        Math.copySign(Math.pow(mPeriodicIO.rotation, 2), mPeriodicIO.rotation),
                        mPeriodicIO.field_relative);
                break;
            case 3:
                // Slew Rate Limiter
                driveSignal = new HolonomicDriveSignal(
                        new Translation2d(
                        forwardLimiter.calculate(mPeriodicIO.forward),
                        strafeLimiter.calculate(mPeriodicIO.strafe)),
                        // rotationLimiter.calculate(mPeriodicIO.rotation),
                        Math.copySign(Math.pow(mPeriodicIO.rotation, 2), mPeriodicIO.rotation),
                        mPeriodicIO.field_relative);
                break;
            default:
                driveSignal = null;
        }
        
        setOpenLoop(driveSignal);
    }

    private void updateModules(HolonomicDriveSignal driveSignal) {
        ChassisSpeeds chassisSpeeds;

        if (driveSignal == null) {
            chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        } else {
            // Convert to velocities and SI units
            var translationInput = driveSignal.getTranslation().times(mSwerveConfiguration.maxSpeedInMetersPerSecond);
            var rotationInput = driveSignal.getRotation() * mSwerveConfiguration.maxSpeedInRadiansPerSecond;

            if (driveSignal.isFieldOriented()) {
                // Adjust for robot heading to maintain field relative motion.
                translationInput = translationInput.rotateBy(getPose().getRotation().unaryMinus());
            }

            chassisSpeeds = new ChassisSpeeds(translationInput.getX(), translationInput.getY(), rotationInput);
        }

        // Now calculate the new Swerve Module states using inverse kinematics.
        mPeriodicIO.swerveModuleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheels speeds if any individual speed is above the specified
        // maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);
    }

    // Method to update Swerve in autonomous
    // Untested and mostly copy-pasted from updateModules()
    // TODO: Look into the SwerveModuleState[] returned from the AutoBuilder and make sure nothing breaks. 
    public synchronized void setSwerveModuleStates(SwerveModuleState[] moduleStates)
    {
        // Update the SwerveModuleStates
        mPeriodicIO.swerveModuleStates = moduleStates;

        // Normalize wheels speeds if any individual speed is above the specified
        // maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);
    }

    public void setAimingTwistScaler(double scaler){
        // if (Double.isNaN(scaler)){
        //     mAimingScaler = kDefaultScaler;
        // }
        // else {
        //     mAimingScaler = scaler;
        // }
        mAimingScaler = kDefaultScaler; // this no longer does anything
    }
    
    private void handleAiming(double timestamp) {
        var dt = timestamp - lastAimTimestamp;
        lastAimTimestamp = timestamp;
        double rotation = 0;

        if (dt > Util.kEpsilon) {
            // mAimingHeaderController.setGoal(Math.toDegrees(mPeriodicIO.visionSetpointInRadians));
            // var rotation = mAimingHeaderController.update();
            
//            mAimingController.setSetpoint(mPeriodicIO.visionSetpointInRadians);
//            double current_angle = getHeading().getDegrees();
//            double current_error = Math.toDegrees(mPeriodicIO.visionSetpointInRadians) - current_angle;
//            if (current_error > 180) {
//                current_angle += 360;
//            } else if (current_error < -180) {
//                current_angle -= 360;
//            }
//
//            var rotation = mAimingController.calculate(Math.toRadians(current_angle), dt);

            // Apply a minimum constant rotation velocity to overcome friction.
            // TODO:  Create constants for these
            // if ((mPeriodicIO.averageWheelVelocity / mSwerveConfiguration.maxSpeedInMetersPerSecond) < 0.2) {
            //     rotation += Math.copySign(0.3 * mSwerveConfiguration.maxSpeedInRadiansPerSecond, rotation);
            // }
            if (Math.abs(rotation) < 0.1) {
                inAimingDeadzone = true;
            } else {
                inAimingDeadzone = false;
            }
            // System.out.println("rot:"+rotation);
            // System.out.println("Swerve.handleAiming() setpoint: "+(((int)(10.0*Math.toDegrees(mPeriodicIO.visionSetpointInRadians)))/10)+ " rotation: "+rotation);
            // Turn in place implies no translational velocity.
            HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
                    new Translation2d(),
                    rotation * mAimingScaler, //2.5,
                    true);

            updateModules(driveSignal);
            //System.out.println("Robot Pose " + mPeriodicIO.robotPose);
        }
    }

    public synchronized void EnableAimingController() {
        // mAimingHeaderController.reset();
        // mAimingHeaderController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
    }

    public synchronized void DisableAimingController() {
        // mAimingHeaderController.setHeadingControllerState(HeadingController.HeadingControllerState.OFF);
    }

    public boolean isInAimingDeadzone() {
        return inAimingDeadzone;
    }

    // //Assigns appropriate directions for scrub factors
    // public void setCarpetDirection(boolean standardDirection) {
    // mModules.forEach((m) -> m.setCarpetDirection(standardDirection));
    // }

    /**
     * Gets the current control state for the Swerve Drive.
     *
     * @return The current control state.
     */
    public synchronized ControlState getState() {
        return mControlState;
    }

    /**
     * Sets the control state for the Swerve Drive.
     *
     * @param newState The desired state.
     */
    public synchronized void setState(ControlState newState) {
        if (mControlState != newState) {
            System.out.println(mControlState + " to " + newState);
            switch (newState) {
                case NEUTRAL:
                    // mPeriodicIO.strafe = 0;
                    // mPeriodicIO.forward = 0;
                    // mPeriodicIO.rotation = 0;
                    // stopSwerveDriveModules();
                    mPeriodicIO.forward = 0.0;
                    mPeriodicIO.strafe = 0.0;
                    mPeriodicIO.rotation = 0.0;
                    mPeriodicIO.visionSetpointInRadians = getHeading().getRadians();
                    mPeriodicIO.schedDeltaDesired = 10; // this is a fast cycle used while testing
                    break;
                case MANUAL:
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 10; // this is a fast cycle used while testing
                    break;

                case VISION_AIM:
                case PATH_FOLLOWING:
                    mPeriodicIO.schedDeltaDesired = 20;
                    break;
            }
        }
        mControlState = newState;
    }

    @Override
    public Command createDefaultCommand(ControllerBinding controllerBinding) {
        return new DefaultSwerveCommand(
            this,
            controllerBinding.supplierFor(StickAction.FORWARD),
            controllerBinding.supplierFor(StickAction.STRAFE),
            controllerBinding.supplierFor(StickAction.ROTATE)
        );
    }

    @Override
    public Pose2d getPose() {
        return mPeriodicIO.robotPose;
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.robotPose.getRotation();
    }

    public ChassisSpeeds getChassisSpeeds() { 
        return mPeriodicIO.chassisSpeeds; 
    }

    /*
     * Get the SwerveDriveKinematics from the SwerveSubsytem.
     */
    @Override
    public SwerveDriveKinematics getKinematics() {
        return mKinematics;
    }

    /**
     * Sets the current robot position on the field.
     *
     * @param pose The (x,y,theta) position.
     */
    @Override
    public void setPose(Pose2d pose) {

        var frontRightPosition = mFrontRight.getPosition();
        var frontLeftPosition = mFrontLeft.getPosition();
        var backLeftPosition = mBackLeft.getPosition();
        var backRightPosition = mBackRight.getPosition();



        mOdometry.resetPosition(mIMU.getYaw(), new SwerveModulePosition []{ frontRightPosition, frontLeftPosition, backLeftPosition, backRightPosition}, pose);
        mPeriodicIO.robotPose = mOdometry.getPoseMeters();
        mGyroOffset = pose.getRotation().rotateBy(Rotation2d.fromDegrees(mIMU.getYaw().getDegrees()).unaryMinus());
    }

    @Override
    public void initForPathFollowing() {
        setState(SwerveSubsystemCurrent.ControlState.PATH_FOLLOWING);
    }

    /**
     * Updates the field relative position of the robot.
     *
     * @param timestamp The current time
     */
    private void updateOdometry(double timestamp) {
        var frontRight = mFrontRight.getState();
        var frontLeft = mFrontLeft.getState();
        var backLeft = mBackLeft.getState();
        var backRight = mBackRight.getState();

        var frontRightPosition = mFrontRight.getPosition();
        var frontLeftPosition = mFrontLeft.getPosition();
        var backLeftPosition = mBackLeft.getPosition();
        var backRightPosition = mBackRight.getPosition();

        // Calculate a threshold for use in aiming
        mPeriodicIO.averageWheelVelocity = (frontLeft.speedMetersPerSecond + frontLeft.speedMetersPerSecond +
                backLeft.speedMetersPerSecond + backRight.speedMetersPerSecond) / 4;

        // order is CCW starting with front right.
        mPeriodicIO.chassisSpeeds = mKinematics.toChassisSpeeds(frontRight, frontLeft, backLeft, backRight);
        mPeriodicIO.robotPose = mOdometry.update(
                mIMU.getYaw(), new SwerveModulePosition []{ frontRightPosition, frontLeftPosition, backLeftPosition, backRightPosition});
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    /**
     * Configure modules for open loop control
     * <p>
     * 
     * @param signal The HolonomicDriveSignal to apply
     */
    public synchronized void setOpenLoop(HolonomicDriveSignal signal) {
        if (mControlState != ControlState.MANUAL) {
            setBrakeMode(true);
            mControlState = ControlState.MANUAL;
        }
        updateModules(signal);
    }

    /**
     * Configure modules for path following.
     * <p>
     * 
     * @param signal The HolonomicDriveSignal to apply
     */
    public synchronized void setPathFollowingVelocity(HolonomicDriveSignal signal) {
        if (mControlState != ControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mControlState = ControlState.PATH_FOLLOWING;
        }
        updateModules(signal);
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mModules.forEach((m) -> m.setNeutralMode(mode));
        }
    }

    /** Puts all steer and drive motors into open-loop mode */
    // public synchronized void disable() {
    //     mModules.forEach((m) -> m.disable());
    //     mControlState = ControlState.DISABLED;
    // }

    /**
     * Zeroes the drive motors, and sets the robot's internal position and heading
     * to match that of the fed pose
     *
     * DO NOT use this to reset the IMU mid match. Use
     * {@link #setRobotPosition(Pose2d)} for that purpose.
     */
    public synchronized void zeroSensors(Pose2d startingPose) {
        setPose(startingPose);
        mIMU.setAngle(startingPose.getRotation().getDegrees());
        //TODO: Determine whether we need this for RobotContainer.getAutonomousCommand
        //mModules.forEach((m) -> m.zeroSensors(startingPose));
    }

    /**
     * Set the setpoint used when aiming the robot for auto shooting.
     *
     * @param setPointInRadians Setpoint in radians
     * @param feedforward Feed-forward term
     * @param timestamp Current timestamp
     */
    public synchronized void setAimingSetpoint(double setPointInRadians, double feedforward, double timestamp) {
        if (mControlState != ControlState.VISION_AIM) {
            mControlState = ControlState.VISION_AIM;
            // seed the last timestamp
            lastAimTimestamp = timestamp;
            // mAimingController.reset();
        }
        mPeriodicIO.visionSetpointInRadians = setPointInRadians;
        mPeriodicIO.visionFeedForward = feedforward;
    }

    /**
     * Sets inputs from driver in teleop mode.
     * <p>
     * 
     * @param forward                percent to drive forwards/backwards (as double
     *                               [-1.0,1.0]).
     * @param strafe                 percent to drive sideways left/right (as double
     *                               [-1.0,1.0]).
     * @param rotation               percent to rotate chassis (as double
     *                               [-1.0,1.0]).
     * @param low_power              whether to use low or high power.
     * @param field_relative         whether operation is robot centric or field
     *                               relative.
     * @param use_heading_controller whether the heading controller is being used.
     */
    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power,
            boolean field_relative, boolean use_heading_controller) {
        if (mControlState != ControlState.MANUAL) {
            mControlState = ControlState.MANUAL;
        }
        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.field_relative = field_relative;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    public synchronized void readPeriodicInputs() {
        double now = lastUpdateTimestamp;
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mIMU.getYaw().getDegrees()).rotateBy(mGyroOffset);
        // mPeriodicIO.gyroYaw = mIMU.getYaw();

        // read modules
        mModules.forEach((m) -> m.readPeriodicInputs());
    }

    public synchronized void writePeriodicOutputs() {
        // Set the module state for each module
        // All modes should use this method of module states.
        if (mControlState != ControlState.NEUTRAL){
            for (int i = 0; i < mModules.size(); i++) {
                mModules.get(i).setState(mPeriodicIO.swerveModuleStates[i]);
            }
        }

        mModules.forEach((m) -> m.writePeriodicOutputs());
    }

    public static class PeriodicIO {
        public TimedState<Pose2dWithCurvature> path_setpoint;
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        private double lastSchedStart;

        // Updated as part of periodic odometry
        public Pose2d robotPose = new Pose2d();
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        // Updated as part of trajectory following
        public Pose2d error = new Pose2d();
//        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());

        // Updated as part of vision aiming
        public double visionSetpointInRadians;
        public double visionFeedForward;
        public double averageWheelVelocity;

        // Inputs
        public Rotation2d gyro_heading = new Rotation2d();
        // public Rotation2d gyroYaw = Rotation2d.identity();
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d())
        };
    }
}
