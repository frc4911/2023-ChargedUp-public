package com.cyberknights4911.robot.model.wham;

import com.cyberknights4911.robot.control.ButtonAction;

/**
 * WHAM! Robot actions that can be bound to button trigger.
 */
public enum WhamButtonAction implements ButtonAction {
    RESET_IMU,
    SLURPP_BACKWARD_FAST,
    SLURPP_FORWARD_FAST,
    CLIMB_WHEEL_LOCK,
    STOW,
    COLLECT_SUBSTATION_BACK,
    COLLECT_SUBSTATION_FRONT,
    SET_GAMEPIECE_CONE,
    SET_GAMEPIECE_CUBE,
    COLLECT_FLOOR_FRONT_CONE,
    SCORE_L2,
    SCORE_L3
}
