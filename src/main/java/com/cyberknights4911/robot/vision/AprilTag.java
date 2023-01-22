package com.cyberknights4911.robot.vision;

/**
 * A simple data class for representing AprilTags.
 */
public final class AprilTag {
    // These are all the tag ids used in the game
    public static final int ID00 = 0;
    public static final int ID01 = 1;
    public static final int ID02 = 2;
    public static final int ID03 = 3;
    public static final int ID04 = 4;
    public static final int ID05 = 5;
    public static final int ID06 = 6;
    public static final int ID07 = 7;

    public final int id;

    public final double xTranslation;
    public final double yTranslation;
    public final double zTranslation;

    public AprilTag(int id, double xTranslation, double yTranslation, double zTranslation) {
        this.id = id;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.zTranslation = zTranslation;
    }
}
