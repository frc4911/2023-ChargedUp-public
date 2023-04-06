package com.cyberknights4911.robot.subsystems.slurpp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the Slurpp, which is responsible for taking in and releasing game
 * pieces.
 */
public final class SlurppSubsystem extends SubsystemBase {

    private final SlurppIO slurppIO;
    private final SlurppIOInputsAutoLogged inputs = new SlurppIOInputsAutoLogged();

    public SlurppSubsystem(SlurppIO splurppIO) {
        super();
        this.slurppIO = splurppIO;
    }

    @Override
    public void periodic() {
        slurppIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Slurpp", inputs);
    }

    /** "Slurpp" up a game piece. */
    public void slurpp(double percentOutput) {
        slurppIO.setPercentOutput(percentOutput);
    }
    
    public void stop() {
        slurppIO.stop();
    }

    public void holdCurrentPosition() {
        slurppIO.holdCurrentPosition();
    }
}
