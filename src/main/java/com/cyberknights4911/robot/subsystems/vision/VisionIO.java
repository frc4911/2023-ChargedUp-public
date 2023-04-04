package com.cyberknights4911.robot.subsystems.vision;

public interface VisionIO {

    public default void subscribeToTag(int tagId, AprilTagTable.OnDetectionInfo listener) {}

    public default void unubscribeToTag(int tagId) {}
}
