package com.cyberknights4911.robot.vision;

import java.util.EnumSet;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;

/**
 * This is responsible for fetching april tags remotely (via NetworTables) and converting them into
 * a robot-readable format.
 */
public final class NetworkMessenger {
    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private final NetworkTableInstance networkTables;
    private final NetworkTable aprilTagTable;
    private final ObjectMapper mapper;

    public NetworkMessenger(NetworkTableInstance networkTables) {
        this.networkTables = networkTables;
        this.aprilTagTable = networkTables.getTable(APRILTAG_TABLE_NAME);
        mapper = new ObjectMapper();
    }

    /**
     * Subscribes to changes for the give AprilTag id.
     * @param tagId the id of the tag.
     */
    public void subscribe(int tagId) {
        StringTopic tagOneTopic = this.aprilTagTable.getStringTopic(String.valueOf(tagId));
        this.networkTables.addListener(
            tagOneTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (tableEvent) -> {
                onTagValue(tagId, tableEvent.valueData.value.getString());
            }
        );
    }
    
    /**
     * This gets called when a tag is detected.
     * @param tagId The id of the tag
     * @param translation The 3d translation of the AprilTag.
     */
    private void onTagValue(int tagId, String tagJson) {
        if (tagJson.isEmpty()) {
            throw new RuntimeException("Invalid tag!");
        }
        try {
            AprilTag foundTag = mapper.readValue(tagJson, AprilTag.class);

            Matrix<N3, N3> camMatrix = new Matrix<N3, N3>(Nat.N3(), Nat.N3());

            for (int i = 0; i < foundTag.getPose_R().length; i++) {
                double[] row = foundTag.getPose_R()[i];
                for (int j = 0; j < row.length; j++) {
                    camMatrix.set(i, j, foundTag.getPose_R()[i][j]);
                }
            }
            Pose3d tPose = new Pose3d(
                new Translation3d(foundTag.getPose_t()[0][0], foundTag.getPose_t()[1][0], foundTag.getPose_t()[2][0]),
                new Rotation3d(camMatrix)
            );
        } catch (JsonProcessingException e) {
            // TODO report this error better
            e.printStackTrace();
        }
    }
}
