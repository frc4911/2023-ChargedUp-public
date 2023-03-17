package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(35, 325),
    COLLECT_SUBSTATION_BACK(245, 160),
    COLLECT_SUBSTATION_FRONT(98, 238),
    COLLECT_FLOOR_FRONT_CONE(38, 263),
    COLLECT_FLOOR_FRONT_CUBE(50, 186),
    COLLECT_FLOOR_BACK_CUBE(320, 117),
    COLLECT_FLOOR_BACK_CONE(320, 105),
    SCORE_L3(230, 195),
    SCORE_L2(95, 219);

    public final double shoulderPosition;
    public final double wristPosition;

    private ArmPositions(double shoulderPosition, double wristPosition) {
        this.shoulderPosition = shoulderPosition;
        this.wristPosition = wristPosition;
    }
}