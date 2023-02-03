package com.cyberknights4911.robot.subsystems.bob;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem for controlling the balancer (Bob).
 */
public final class BobSubsystem implements Subsystem {
    private static final double TICKS_AT_EXTENSION = 10000; //Random value
    
    private final BobIO bobIO;

    public BobSubsystem(BobIO bobIO) {
        this.bobIO = bobIO;
    }

    public void setLockout(boolean extend) {
        bobIO.setExtended(extend);
    }

    public void toggleBob(boolean extend) {
        double desiredPosition = 0;
        if (extend) {
            desiredPosition = TICKS_AT_EXTENSION;
        }
        bobIO.setPosition(desiredPosition);
    }

}