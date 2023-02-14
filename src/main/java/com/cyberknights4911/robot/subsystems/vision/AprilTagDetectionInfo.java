package com.cyberknights4911.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;

/**
 * Wrapper for keeping detection info for a detected AprilTag.
 */
public final class AprilTagDetectionInfo {
    private final double hamming;
    private final double decistionMargin;
    private final double[] center;
    private final double[] corners;
    private final AprilTag aprilTag;

    public AprilTagDetectionInfo(
        double hamming,
        double decistionMargin,
        double[] center,
        double[] corners,
        AprilTag aprilTag
    ) {
        this.hamming = hamming;
        this.decistionMargin = decistionMargin;
        this.center = center;
        this.corners = corners;
        this.aprilTag = aprilTag;
    }

    public double getHamming() {
        return hamming;
    }

    public double getDecistionMargin() {
        return decistionMargin;
    }

    public double[] getCenter() {
        return center;
    }

    public double[] getCorners() {
        return corners;
    }

    public AprilTag getAprilTag() {
        return aprilTag;
    }    
}
