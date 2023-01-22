package com.cyberknights4911.robot.vision;

import java.util.EnumSet;

import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This is responsible for fetching april tags remotely (via NetworTables) and converting them into
 * a robot-readable format.
 */
public final class NetworkMessenger {
    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private final NetworkTableInstance networkTables;
    private final NetworkTable aprilTagTable;

    public NetworkMessenger(NetworkTableInstance networkTables) {
        this.networkTables = networkTables;
        this.aprilTagTable = networkTables.getTable(APRILTAG_TABLE_NAME);
    }

    /**
     * Subscribes to changes for the give AprilTag id.
     * @param tagId the id of the tag.
     */
    public void subscribe(int tagId) {
        DoubleArrayTopic tagOneTopic = this.aprilTagTable.getDoubleArrayTopic(String.valueOf(tagId));
        this.networkTables.addListener(
            tagOneTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (tableEvent) -> {
                onTagValue(tagId, tableEvent.valueData.value.getDoubleArray());
            }
        );
    }
    
    /**
     * This gets called when a tag is detected.
     * @param tagId The id of the tag
     * @param translation The 3d translation of the AprilTag.
     */
    private void onTagValue(int tagId, double[] translation) {
        if (translation.length != 3) {
            throw new RuntimeException("Invalid tag!");
        }
        AprilTag foundTag = new AprilTag(tagId, translation[0], translation[1], translation[2]);
        // TODO: Do something depending on the state of the robot and the tag id.
    }
}
