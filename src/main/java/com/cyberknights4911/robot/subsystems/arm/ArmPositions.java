package com.cyberknights4911.robot.subsystems.arm;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.DoublePreference;

public enum ArmPositions {
    STOWED(
        Constants.Arm.STOWED_SHOULDER,
        Constants.Arm.STOWED_WRIST),
    COLLECT_SINGLE_SUBSTATION_FRONT(
        Constants.Arm.COLLECT_SINGLE_SUBSTATION_FRONT_SHOULDER, 
        Constants.Arm.COLLECT_SINGLE_SUBSTATION_FRONT_WRIST),
    COLLECT_SUBSTATION_BACK(
        Constants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER,
        Constants.Arm.COLLECT_SUBSTATION_BACK_WRIST),
    COLLECT_SUBSTATION_FRONT(
        Constants.Arm.COLLECT_SUBSTATION_FRONT_SHOULDER, 
        Constants.Arm.COLLECT_SUBSTATION_FRONT_WRIST),
    COLLECT_FLOOR_FRONT_CONE(
        Constants.Arm.COLLECT_FLOOR_FRONT_CONE_SHOULDER, 
        Constants.Arm.COLLECT_FLOOR_FRONT_CONE_WRIST),
    COLLECT_FLOOR_FRONT_CUBE(
        Constants.Arm.COLLECT_FLOOR_FRONT_CUBE_SHOULDER, 
        Constants.Arm.COLLECT_FLOOR_FRONT_CUBE_WRIST),
    COLLECT_FLOOR_BACK_CUBE(
        Constants.Arm.COLLECT_FLOOR_BACK_CUBE_SHOULDER, 
        Constants.Arm.COLLECT_FLOOR_BACK_CUBE_WRIST),
    COLLECT_FLOOR_BACK_CONE(
        Constants.Arm.COLLECT_FLOOR_BACK_CONE_SHOULDER, 
        Constants.Arm.COLLECT_FLOOR_BACK_CONE_WRIST),
    SCORE_L3(
        Constants.Arm.SCORE_L3_SHOULDER, 
        Constants.Arm.SCORE_L3_WRIST),
    SCORE_L2(
        Constants.Arm.SCORE_L2_SHOULDER, 
        Constants.Arm.SCORE_L2_WRIST);

    public final DoublePreference shoulderPosition;
    public final DoublePreference wristPosition;

    private ArmPositions(DoublePreference shoulderPosition, DoublePreference wristPosition) {
        this.shoulderPosition = shoulderPosition;
        this.wristPosition = wristPosition;
    }
}