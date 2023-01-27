package com.cyberknights4911.robot.vision;

import java.util.EnumSet;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            // TODO: Do something depending on the state of the robot and the tag id.
            Pose2d tagPose = new Pose2d(
                new Translation2d(foundTag.getPose_t()[0][0], foundTag.getPose_t()[1][0]),
                new Rotation2d(foundTag.getPose_R()[0][0], foundTag.getPose_R()[1][0])
            );
        } catch (JsonProcessingException e) {
            // TODO report this error better
            e.printStackTrace();
        }
    }
}
