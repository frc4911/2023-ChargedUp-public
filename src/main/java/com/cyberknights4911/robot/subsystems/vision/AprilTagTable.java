package com.cyberknights4911.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class used to get the latest detection info for an AprilTag.
 */
public final class AprilTagTable {
    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private static final String SUB_TABLE_NAME_PREFIX = "tag_%d";
    private static final String DETECTED_TOPIC_NAME = "detected";
    private static final String HAMMING_TOPIC_NAME = "hamming";
    private static final String DECISION_MARGIN_TOPIC_NAME = "decision_margin";
    private static final String CENTER_TOPIC_NAME = "center";
    private static final String CORNERS_TOPIC_NAME = "corners";
    private static final String POSE_R_TOPIC_NAME = "pose_R";
    private static final String POSE_T_TOPIC_NAME = "pose_t";
    
    private final int tagId;
    private final NetworkTable tagSubtable;
    private final BooleanSubscriber detected;
    private final DoubleSubscriber hamming;
    private final DoubleSubscriber decisionMargin;
    private final DoubleArraySubscriber center;
    private final DoubleArraySubscriber corners;
    private final DoubleArraySubscriber pose_R;
    private final DoubleArraySubscriber pose_t;

    public AprilTagTable(int tagId, NetworkTableInstance networkTables) {
        this.tagId = tagId;
        tagSubtable = networkTables.getTable(APRILTAG_TABLE_NAME)
            .getSubTable(String.format(SUB_TABLE_NAME_PREFIX, tagId));
        detected = tagSubtable.getBooleanTopic(DETECTED_TOPIC_NAME).subscribe(false);
        hamming = tagSubtable.getDoubleTopic(HAMMING_TOPIC_NAME).subscribe(0.0);
        decisionMargin = tagSubtable.getDoubleTopic(DECISION_MARGIN_TOPIC_NAME).subscribe(0.0);
        center = tagSubtable.getDoubleArrayTopic(CENTER_TOPIC_NAME).subscribe(new double[0]);
        corners = tagSubtable.getDoubleArrayTopic(CORNERS_TOPIC_NAME).subscribe(new double[0]);
        pose_R = tagSubtable.getDoubleArrayTopic(POSE_R_TOPIC_NAME).subscribe(new double[0]);
        pose_t = tagSubtable.getDoubleArrayTopic(POSE_T_TOPIC_NAME).subscribe(new double[0]);
    }

    public Optional<AprilTagDetectionInfo> getLatestAprilTag() {
        if (!detected.get()) {
            // Any other fields are from a previous detection. Ignore it.
            return Optional.empty();
        }

        double hammingValue = hamming.get();
        double decisionMarginValue = decisionMargin.get();
        double[] rotationPoseMatrix = pose_R.get();
        double[] translationMatrix = pose_t.get();
        double[] centerArray = center.get();
        double[] cornerArray = corners.get();

        if (rotationPoseMatrix.length < 9) {
            return Optional.empty();
        } else if (translationMatrix.length < 3) {
            return Optional.empty();
        }

        Matrix<N3, N3> camMatrix = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        // pose_R is a flattened 3x3 "camera matrix"
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camMatrix.set(i, j, rotationPoseMatrix[i+j+i*i]);
            }
        }

        return Optional.of(
            new AprilTagDetectionInfo(
                hammingValue,
                decisionMarginValue,
                centerArray,
                cornerArray,
                new AprilTag(
                    tagId,
                    new Pose3d(
                        new Translation3d(translationMatrix[0], translationMatrix[1], translationMatrix[2]),
                        new Rotation3d(camMatrix)
                    )
                )
            )
        );
    }
}
