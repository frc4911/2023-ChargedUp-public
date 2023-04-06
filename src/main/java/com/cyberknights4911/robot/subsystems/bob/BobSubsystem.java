package com.cyberknights4911.robot.subsystems.bob;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the balancer (Bob).
 */
public final class BobSubsystem extends SubsystemBase {
    private static final double TICKS_AT_EXTENSION = 10000; //Random value
    
    private final BobIO bobIO;
    private final BobIOInputsAutoLogged inputs = new BobIOInputsAutoLogged();

    public BobSubsystem(BobIO bobIO) {
        super();
        this.bobIO = bobIO;
    }

    @Override
    public void periodic() {
        // bobIO.updateInputs(inputs);
        // Logger.getInstance().processInputs("Bob", inputs);
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