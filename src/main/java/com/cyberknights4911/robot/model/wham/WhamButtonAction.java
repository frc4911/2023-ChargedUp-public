package com.cyberknights4911.robot.model.wham;

import com.cyberknights4911.robot.control.ButtonAction;

/**
 * WHAM! Robot actions that can be bound to button trigger.
 */
public enum WhamButtonAction implements ButtonAction {
    ALIGN_COLLECT,
    RESET_IMU,
    SLURPP_BACKWARD_FAST,
    SLURPP_FORWARD_FAST,
    SLURPP_BACKWARD_SLOW,
    SLURPP_FORWARD_SLOW,
    CLIMB_WHEEL_LOCK,
    RESET_WHEELS,
    CLIMB_DEPLOY,
    CLIMB_LOCKOUT,
    BOB_STOW,
    BOB_DEPLOY,
    STOW,
    COLLECT_SINGLE_SUBSTATION_FRONT,
    COLLECT_SUBSTATION_BACK,
    COLLECT_SUBSTATION_FRONT,
    COLLECT_FLOOR_FRONT_CUBE,
    COLLECT_FLOOR_FRONT_CONE,
    COLLECT_FLOOR_BACK_CUBE,
    COLLECT_FLOOR_BACK_CONE,
    SCORE_L2,
    SCORE_L3,
    HOME,
    HOME_CLAW
}