package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(35, 325),
    COLLECT_SUBSTATION_BACK(249, 160),
    COLLECT_SUBSTATION_FRONT(105, 205),
    COLLECT_FLOOR_FRONT_CONE(35, 256),
    COLLECT_FLOOR_FRONT_CUBE(50, 186),
    COLLECT_FLOOR_BACK_CUBE(320, 117),
    COLLECT_FLOOR_BACK_CONE(320, 105),
    SCORE_L3(230, 195),
    SCORE_L2(93, 220),
    // These are for preventing height violations. Don't use them directly
    INTERMEDIATE_FRONT(110, 300),
    INTERMEDIATE_BACK(240, 300);

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