package com.cyberknights4911.robot.vision;

import java.util.EnumSet;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;

/**
 * This is responsible for receiving april tags from the vision coprocessor and converting them
 * into a robot-consumable format.
 */
public final class AprilTagCoprocessorMessenger {
    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private static final String APRILTAG_ENTRY_NAME = "DETECTED_TAGS";

    private final NetworkTableInstance networkTables;
    private final NetworkTable aprilTagTable;
    private final ObjectMapper mapper;
    private int listenerId = -1;

    public AprilTagCoprocessorMessenger(NetworkTableInstance networkTables) {
        this.networkTables = networkTables;
        this.aprilTagTable = networkTables.getTable(APRILTAG_TABLE_NAME);
        mapper = new ObjectMapper();
    }

    /**
     * Subscribes to changes for detected AprilTags.
     */
    public void subscribe() {
        StringTopic tagOneTopic = this.aprilTagTable.getStringTopic(APRILTAG_ENTRY_NAME);
        listenerId = this.networkTables.addListener(
            tagOneTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (tableEvent) -> {
                try {
                    AprilTag[] tags = onTagsMessage(tableEvent.valueData.value.getString());
                    for (AprilTag tag: tags) {
                        System.out.println("tag: " + tag.toString());
                    }
                } catch (JsonProcessingException|IllegalArgumentException e) {
                    e.printStackTrace();
                }
            }
        );
    }

    /**
     * Unsubscribes to changes for detected AprilTags.
     */
    public void unsubscribe() {
        if (listenerId != -1) {
            this.networkTables.removeListener(listenerId);
        }
    }
    
    /**
     * This gets called when a tag mesage is detected. The message is an array of all tags that
     * were detected from the last image the coprocessor checked.
     * @param tagJson json message holding an array of tags.
     */
    private AprilTag[] onTagsMessage(String tagJson) throws JsonProcessingException {
        if (tagJson.isEmpty()) {
            throw new IllegalArgumentException("Invalid tag message!");
        }
        // 1. Read the json message into an array of java objects
        AprilTagJsonMessage[] foundTags = mapper.readValue(tagJson, AprilTagJsonMessage[].class);
        
        // 2. Convert to an array of native AprilTag objects
        AprilTag[] wpiTags = new AprilTag[foundTags.length];
        for (int i = 0; i < foundTags.length; i++) {
            wpiTags[i] = foundTags[i].convertToWpiAprilTag();
        }
        return wpiTags;
    }
}
