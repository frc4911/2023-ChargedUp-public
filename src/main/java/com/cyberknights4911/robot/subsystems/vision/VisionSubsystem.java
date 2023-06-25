package com.cyberknights4911.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for handling vision.
 */
public class VisionSubsystem extends SubsystemBase implements AprilTagTable.OnDetectionInfo {
    private final VisionIO visionIO;
    private final Object LISTENER_LOCK = new Object();

    private PoseEstimateListener listener;

    public VisionSubsystem(VisionIO visionIO) {
        this.visionIO = visionIO;
    }

    // TODO: call this with the correct alliance tags at the start of auto
    public void subscribe(int... tagIds) {
        for (int tagId : tagIds) {
            visionIO.subscribeToTag(tagId, this);
        }
    }

    // TODO: call this with the correct alliance tags at the start of teleop
    public void unsubscribe(int... tagIds) {
        for (int tagId : tagIds) {
            visionIO.unubscribeToTag(tagId);
        }
    }

    public interface PoseEstimateListener {
        /**
         * Called when a new pose estimate is received from the vision system
         * @param pose2d the estimaged pose, in meters
         * @param timeStampSeconds the time the pose was estimated
         */
        void onPoseEstimate(Pose2d pose2d, double timestampSeconds);
    }

    @Override
    public void onDetectionInfo(AprilTagDetectionInfo info) {
        synchronized(LISTENER_LOCK) {
            if (listener == null) {
                return;
            }

            // TODO: convert info into robot pose, then pass it to estimator
        }
    }
    

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
