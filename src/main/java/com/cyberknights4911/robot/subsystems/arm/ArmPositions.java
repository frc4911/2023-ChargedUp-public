package com.cyberknights4911.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.cyberknights4911.robot.constants.Constants.PROFILE_ARM_SPEED_STOPPED;
import static com.cyberknights4911.robot.constants.Constants.PROFILE_ARM_SPEED_FORWARD;
import static com.cyberknights4911.robot.constants.Constants.PROFILE_ARM_SPEED_BACKWARD;

public enum ArmPositions {
    STOWED(35, 325),
    COLLECT_SUBSTATION_BACK(245, 160),
    COLLECT_SUBSTATION_FRONT(98, 238),
    COLLECT_FLOOR_FRONT_CONE(35, 263),
    COLLECT_FLOOR_FRONT_CUBE(50, 186),
    COLLECT_FLOOR_BACK_CUBE(320, 117),
    COLLECT_FLOOR_BACK_CONE(320, 105),
    SCORE_L3(230, 195),
    SCORE_L2(95, 219),
    // These are for preventing height violations. Don't use them directly
    INTERMEDIATE_FRONT_FROM_FRONT(140, 250, PROFILE_ARM_SPEED_BACKWARD),
    INTERMEDIATE_BACK_FROM_FRONT(210, 250, PROFILE_ARM_SPEED_BACKWARD),
    INTERMEDIATE_FRONT_FROM_BACK(140, 250, PROFILE_ARM_SPEED_FORWARD),
    INTERMEDIATE_BACK_FROM_BACK(210, 250, PROFILE_ARM_SPEED_FORWARD);

    public final TrapezoidProfile.State shoulderState;
    public final TrapezoidProfile.State wristState;

    private ArmPositions(double shoulderPosition, double wristPosition) {
        this.shoulderState =
            new TrapezoidProfile.State(shoulderPosition, PROFILE_ARM_SPEED_STOPPED);
        this.wristState = new TrapezoidProfile.State(wristPosition, PROFILE_ARM_SPEED_STOPPED);
    }

    private ArmPositions(
        double shoulderPosition, double wristPosition, double shoulderVelocity
    ) {
        this.shoulderState = new TrapezoidProfile.State(shoulderPosition, shoulderVelocity);
        this.wristState = new TrapezoidProfile.State(wristPosition, PROFILE_ARM_SPEED_STOPPED);
    }
}