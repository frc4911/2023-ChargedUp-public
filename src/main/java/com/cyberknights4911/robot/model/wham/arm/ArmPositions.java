package com.cyberknights4911.robot.model.wham.arm;

import com.cyberknights4911.robot.model.wham.WhamConstants;
import com.cyberknights4911.robot.constants.DoublePreference;

public enum ArmPositions {
    STOWED(
        WhamConstants.Arm.STOWED_SHOULDER,
        WhamConstants.Arm.STOWED_WRIST),
    COLLECT_SINGLE_SUBSTATION_FRONT(
        WhamConstants.Arm.COLLECT_SINGLE_SUBSTATION_FRONT_SHOULDER, 
        WhamConstants.Arm.COLLECT_SINGLE_SUBSTATION_FRONT_WRIST),
    COLLECT_SUBSTATION_BACK(
        WhamConstants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER,
        WhamConstants.Arm.COLLECT_SUBSTATION_BACK_WRIST),
    COLLECT_SUBSTATION_FRONT(
        WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_SHOULDER, 
        WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_WRIST),
    COLLECT_FLOOR_FRONT_CONE(
        WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_SHOULDER, 
        WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_WRIST),
    COLLECT_FLOOR_FRONT_CUBE(
        WhamConstants.Arm.COLLECT_FLOOR_FRONT_CUBE_SHOULDER, 
        WhamConstants.Arm.COLLECT_FLOOR_FRONT_CUBE_WRIST),
    COLLECT_FLOOR_BACK_CUBE(
        WhamConstants.Arm.COLLECT_FLOOR_BACK_CUBE_SHOULDER, 
        WhamConstants.Arm.COLLECT_FLOOR_BACK_CUBE_WRIST),
    COLLECT_FLOOR_BACK_CONE(
        WhamConstants.Arm.COLLECT_FLOOR_BACK_CONE_SHOULDER, 
        WhamConstants.Arm.COLLECT_FLOOR_BACK_CONE_WRIST),
    SCORE_L3(
        WhamConstants.Arm.SCORE_L3_SHOULDER, 
        WhamConstants.Arm.SCORE_L3_WRIST),
    SCORE_L2(
        WhamConstants.Arm.SCORE_L2_SHOULDER, 
        WhamConstants.Arm.SCORE_L2_WRIST);

    public final DoublePreference shoulderPosition;
    public final DoublePreference wristPosition;

    ArmPositions(DoublePreference shoulderPosition, DoublePreference wristPosition) {
        this.shoulderPosition = shoulderPosition;
        this.wristPosition = wristPosition;
    }
}