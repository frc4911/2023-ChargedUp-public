package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    //Values are in degrees and later converted to ticks for ease of comprehension
    //0 Degrees are at stowed for both SHOULDER and WRIST
    STOWED(42, 42),
    CONE_LEVEL_3(220, 100), 
    CUBE_LEVEL_3(220, 100),
    CONE_LEVEL_2(50, 100),
    CUBE_LEVEL_2(50, 100),
    HYBRID_CONE(300, 42),
    HYBRID_CUBE(300, 42),
    COLLECT_PORTAL(240, 42),
    COLLECT_GROUND(300, 42);

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