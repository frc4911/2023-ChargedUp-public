package com.cyberknights4911.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class used to get the latest detection info for an AprilTag.
 */
public final class AprilTagTable {
    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private static final String SUB_TABLE_NAME_PREFIX = "tag_%d";
    private static final String VALUES_TOPIC_NAME = "values";
    
    // All the tag info is packed into one double array
    private static final int START_INDEX_DETECTED = 0; // length of 1
    private static final int START_INDEX_HAMMING = 1; // length of 1
    private static final int START_INDEX_DECISION_MARGIN = 2; // length of 1
    private static final int START_INDEX_CENTER = 3; // length of 2
    private static final int START_INDEX_CORNERS = 5; // length of 8
    private static final int START_INDEX_POSE_R = 13; // length of 9
    private static final int START_INDEX_POSE_T = 22; // length of 3

    private static final int ARRAY_LENGTH_CENTER = 2;
    private static final int ARRAY_LENGTH_CORNERS = 8;
    private static final int ARRAY_LENGTH_POSE_R = 9;    
    private static final int ARRAY_LENGTH_POSE_T = 3;
    
    private final int tagId;
    private final NetworkTable tagSubtable;
    private final DoubleArraySubscriber tagValuesSubscriber;

    public AprilTagTable(int tagId, NetworkTableInstance networkTables) {
        this.tagId = tagId;
        tagSubtable = networkTables.getTable(APRILTAG_TABLE_NAME)
            .getSubTable(String.format(SUB_TABLE_NAME_PREFIX, tagId));
        tagValuesSubscriber = tagSubtable.getDoubleArrayTopic(VALUES_TOPIC_NAME).subscribe(new double[0]);
    }

    public Optional<AprilTagDetectionInfo> getLatestAprilTag() {
        double[] valuesArray = tagValuesSubscriber.get();
        if (valuesArray.length == 0) {
            // Nothing is there
            return Optional.empty();
        }
        if (valuesArray[START_INDEX_DETECTED] == 0.0) {
            // Any other fields are from a previous detection. Ignore it.
            return Optional.empty();
        }

        double hammingValue = valuesArray[START_INDEX_HAMMING];
        double decisionMarginValue = valuesArray[START_INDEX_DECISION_MARGIN];
        double[] centerArray = Arrays.copyOfRange(
            valuesArray,
            START_INDEX_CENTER,
            ARRAY_LENGTH_CENTER);
        double[] cornersArray = Arrays.copyOfRange(
            valuesArray,
            START_INDEX_CORNERS,
            ARRAY_LENGTH_CORNERS);

        Matrix<N3, N3> camMatrix = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        // pose_R is a flattened 3x3 "camera matrix"
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camMatrix.set(i, j, valuesArray[START_INDEX_POSE_R + i + j + 2 * i]);
            }
        }
        return Optional.of(
            new AprilTagDetectionInfo(
                hammingValue,
                decisionMarginValue,
                centerArray,
                cornersArray,
                new AprilTag(
                    tagId,
                    new Pose3d(
                        new Translation3d(
                            valuesArray[START_INDEX_POSE_T],
                            valuesArray[START_INDEX_POSE_T + 1],
                            valuesArray[START_INDEX_POSE_T + 2]
                        ),
                        new Rotation3d(camMatrix)
                    )
                )
            )
        );
    }
}
