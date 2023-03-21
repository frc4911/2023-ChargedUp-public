package com.cyberknights4911.robot.subsystems.arm;

import com.cyberknights4911.robot.constants.Constants;

public enum ArmPositions {
    STOWED(
        Constants.Arm.STOWED_SHOULDER.getValue(),
        Constants.Arm.STOWED_WRIST.getValue()),
    COLLECT_SUBSTATION_BACK(
        Constants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER.getValue(),
        Constants.Arm.COLLECT_SUBSTATION_BACK_WRIST.getValue()),
    COLLECT_SUBSTATION_FRONT(
        Constants.Arm.COLLECT_SUBSTATION_FRONT_SHOULDER.getValue(), 
        Constants.Arm.COLLECT_SUBSTATION_FRONT_WRIST.getValue()),
    COLLECT_FLOOR_FRONT_CONE(
        Constants.Arm.COLLECT_FLOOR_FRONT_CONE_SHOULDER.getValue(), 
        Constants.Arm.COLLECT_FLOOR_FRONT_CONE_WRIST.getValue()),
    COLLECT_FLOOR_FRONT_CUBE(
        Constants.Arm.COLLECT_FLOOR_FRONT_CUBE_SHOULDER.getValue(), 
        Constants.Arm.COLLECT_FLOOR_FRONT_CUBE_WRIST.getValue()),
    COLLECT_FLOOR_BACK_CUBE(
        Constants.Arm.COLLECT_FLOOR_BACK_CUBE_SHOULDER.getValue(), 
        Constants.Arm.COLLECT_FLOOR_BACK_CUBE_WRIST.getValue()),
    COLLECT_FLOOR_BACK_CONE(
        Constants.Arm.COLLECT_FLOOR_BACK_CONE_SHOULDER.getValue(), 
        Constants.Arm.COLLECT_FLOOR_BACK_CONE_WRIST.getValue()),
    SCORE_L3(
        Constants.Arm.SCORE_L3_SHOULDER.getValue(), 
        Constants.Arm.SCORE_L3_WRIST.getValue()),
    SCORE_L2(
        Constants.Arm.SCORE_L2_SHOULDER.getValue(), 
        Constants.Arm.SCORE_L2_WRIST.getValue());

    public final double shoulderPosition;
    public final double wristPosition;

    private ArmPositions(double shoulderPosition, double wristPosition) {
        this.shoulderPosition = shoulderPosition;
        this.wristPosition = wristPosition;
    }
}