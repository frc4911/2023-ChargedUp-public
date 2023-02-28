package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(35, 325),
    COLLECT_SUBSTATION_BACK(256, 167),
    COLLECT_SUBSTATION_FRONT(101, 198),
    COLLECT_FLOOR_FRONT_CONE(35, 256),
    COLLECT_FLOOR_FRONT_CUBE(50, 186),
    COLLECT_FLOOR_BACK_CUBE(320, 117),
    COLLECT_FLOOR_BACK_CONE(320, 105),
    SCORE_L3(238, 204),
    SCORE_L2(89, 216);

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