package com.cyberknights4911.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This is responsible for fetching april tags remotely (via NetworTables) and converting them into
 * a robot-readable format.
 */
public final class NetworkMessenger {

    private static final String APRILTAG_TABLE_NAME = "APRIL_TAGS";
    private static final String[] ALL_TAG_IDS = {
        AprilTagIds.ID00, AprilTagIds.ID01, AprilTagIds.ID02, AprilTagIds.ID03, AprilTagIds.ID04,
        AprilTagIds.ID05, AprilTagIds.ID06, AprilTagIds.ID07, AprilTagIds.ID08, AprilTagIds.ID09,
        AprilTagIds.ID10, AprilTagIds.ID11
    };

    private final NetworkTableInstance networkTables;

    private final NetworkTable aprilTagTable;

    public NetworkMessenger(NetworkTableInstance networkTables) {
        this.networkTables = networkTables;
        this.aprilTagTable = networkTables.getTable(APRILTAG_TABLE_NAME);
    }

    public void getTagInfo() {

    }

    private interface AprilTagIds {
        String ID00 = "id00";
        String ID01 = "id01";
        String ID02 = "id02";
        String ID03 = "id03";
        String ID04 = "id04";
        String ID05 = "id05";
        String ID06 = "id06";
        String ID07 = "id07";
        String ID08 = "id08";
        String ID09 = "id09";
        String ID10 = "id10";
        String ID11 = "id11";
    }
    
}
