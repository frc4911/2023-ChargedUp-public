package com.cyberknights4911.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;

import com.cyberknights4911.robot.subsystems.vision.AprilTagTable.OnDetectionInfo;

import edu.wpi.first.networktables.NetworkTableInstance;

public final class VisionIOReal implements VisionIO {
    private final Map<Integer, AprilTagTable> tagTables = new HashMap<>();
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();

    @Override
    public void subscribeToTag(int tagId, OnDetectionInfo listener) {
        AprilTagTable table = new AprilTagTable(tagId, networkTables);
        tagTables.put(tagId, table);
        table.listen(listener);
    }

    @Override
    public void unubscribeToTag(int tagId) {
        AprilTagTable table = tagTables.remove(tagId);
        if (table != null) {
            table.stop();
        }
    }
    
}
