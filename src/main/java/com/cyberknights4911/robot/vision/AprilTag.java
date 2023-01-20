package com.cyberknights4911.robot.vision;

/**
 * A simple data class for representing AprilTags.
 */
public final class AprilTag {
    public final int id;

    public final double xTranslation;
    public final double yTranslation;
    public final double zTranslation;

    public final double xRotation;
    public final double yRotation;
    public final double zRotation;

    public AprilTag(int id, double xTranslation, double yTranslation, double zTranslation,
            double xRotation, double yRotation, double zRotation) {
        this.id = id;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.zTranslation = zTranslation;
        this.xRotation = xRotation;
        this.yRotation = yRotation;
        this.zRotation = zRotation;
    }
}
