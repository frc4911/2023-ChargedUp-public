package com.cyberknights4911.robot.model.wham.arm;

import com.cyberknights4911.robot.model.wham.WhamConstants;
import com.cyberknights4911.robot.constants.DoublePreference;

public enum ArmPositions {
    STOWED,
    COLLECT_SUBSTATION_BACK,
    COLLECT_SUBSTATION_FRONT,
    COLLECT_FLOOR_FRONT_CONE,
    SCORE_L3,
    SCORE_L2;

    public DoublePreference getShoulderPosition() {
        switch (this) {
            case STOWED: return WhamConstants.Arm.STOWED_SHOULDER;
            case COLLECT_SUBSTATION_BACK: return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER;
            case COLLECT_SUBSTATION_FRONT: return WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_SHOULDER;
            case COLLECT_FLOOR_FRONT_CONE: return WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_SHOULDER;
            case SCORE_L3: return WhamConstants.Arm.SCORE_L3_SHOULDER;
            case SCORE_L2: return WhamConstants.Arm.SCORE_L2_SHOULDER;
            default: return WhamConstants.Arm.STOWED_SHOULDER;
        }
    }

    public DoublePreference getWristPosition() {
        switch (this) {
            case STOWED: return WhamConstants.Arm.STOWED_WRIST;
            case COLLECT_SUBSTATION_BACK: return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_WRIST;
            case COLLECT_SUBSTATION_FRONT: return WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_WRIST;
            case COLLECT_FLOOR_FRONT_CONE: return WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_WRIST;
            case SCORE_L3: return WhamConstants.Arm.SCORE_L3_WRIST;
            case SCORE_L2: return WhamConstants.Arm.SCORE_L2_WRIST;
            default: return WhamConstants.Arm.STOWED_WRIST;
        }
    }
}