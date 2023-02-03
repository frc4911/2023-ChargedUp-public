package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    //TODO: These values are untuned. Need Testing
    //Values are in degrees and later converted to ticks for ease of comprehension
    //0 Degrees are at stowed for both SHOULDER and WRIST
    STOWED(0, 0),
    CONE_LEVEL_3(240, 100), 
    CUBE_LEVEL_3(240, 100),
    CONE_LEVEL_2(270, 100),
    CUBE_LEVEL_2(270, 100),
    HYBRID_CONE(300, 100),
    HYBRID_CUBE(300, 100),
    COLLECT_PORTAL(240, 100),
    COLLECT_GROUND(300, 100);

    public final double shoulderPosition;
    public final double wristPosition;

    private ArmPositions(double shoulderPosition, double wristPosition) {
        this.shoulderPosition = shoulderPosition;
        this.wristPosition = wristPosition;
    }

    public double getShoulderPosition() {
        return shoulderPosition;
    }

    public double getWristPosition() {
        return wristPosition;
    }
}