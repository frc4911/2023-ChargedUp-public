package com.cyberknights4911.robot.subsystems.arm;

public enum ArmPositions {
    STOWED(35, 325), // actual value
    COLLECT_SUBSTATION_BACK(256, 167), // actual values
    COLLECT_SUBSTATION_FRONT(101, 198), // actual values
    SCORE_L3(238, 204), // actual values
    // TODO: determine the rest of these by positioning the arm and checking shuffleboard
    SCORE_L2(42, 42),
    COLLECT_FLOOR_FRONT_CUBE(42, 42),
    COLLECT_FLOOR_FRONT_CONE(42, 42),
    COLLECT_FLOOR_BACK_CUBE(42, 42),
    COLLECT_FLOOR_BACK_CONE(42, 42);

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