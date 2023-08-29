package com.cyberknights4911.robot.model.quickdrop;

import com.cyberknights4911.robot.control.ButtonAction;

/**
 * Quickdrop robot actions that can be bound to button trigger.
 */
public enum QuickDropButtonAction implements ButtonAction {
    RESET_IMU,
    COLLECTOR_EXTEND,
    COLLECTOR_RETRACT,
    COLLECTOR_RUN_FORWARD,
    COLLECTOR_RUN_REVERSE,
    INDEXER_RUN,
}
