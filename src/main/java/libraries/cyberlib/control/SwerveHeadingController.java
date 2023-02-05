package libraries.cyberlib.control;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;

import libraries.cheesylib.util.SynchronousPIDF;

/**
 * Controls overall swerve heading of the robot.
 */
public class SwerveHeadingController {
    private static SwerveHeadingController mInstance;
    private static SwerveSubsystemCurrent mSwerve = null;

    public static SwerveHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveHeadingController();
            // this needs to happen during class creation but
            // after mInstance is set because Swerve will get an
            // instance of SwerveHeadingController
            //mSwerve = Swerve.getInstance("SwerveHeadingController");
        }

        return mInstance;
    }

    public enum HeadingControllerState {
        OFF,
        // SNAP, // for dpad snapping to cardinals
        MAINTAIN, // maintaining current heading while driving
    }

    private final SynchronousPIDF mPIDFController;
    private double mSetpoint = 0.0;

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private SwerveHeadingController() {
        mPIDFController = new SynchronousPIDF();
    }

    public void setPIDFConstants(double kP, double kI, double kD, double kF) {
        mPIDFController.setPIDF(kP, kI, kD, kF);
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * Sets the goal position.
     * <p>
     * 
     * @param goalPositionInDegrees goal position in degrees
     */
    public void setGoal(double goalPositionInDegrees) {
        mSetpoint = goalPositionInDegrees;
    }

    public boolean isAtGoal() {
        return mPIDFController.onTarget(Constants.SWERVE_HEADING_CONTROLLER_ERROR_TOLERANCE);
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update() {
        mPIDFController.setSetpoint(mSetpoint);
        // double current_angle =
        // Swerve.getInstance("SwerveHeadingController").getHeading().getDegrees();
        double current_angle = mSwerve.getHeading().getDegrees();
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        if (mHeadingControllerState == HeadingControllerState.OFF) {
            return 0;
        }

        return mPIDFController.calculate(current_angle);
    }
}