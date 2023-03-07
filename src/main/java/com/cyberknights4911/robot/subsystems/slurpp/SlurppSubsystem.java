package com.cyberknights4911.robot.subsystems.slurpp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the Slurpp, which is responsible for taking in and releasing game
 * pieces.
 */
public final class SlurppSubsystem extends SubsystemBase {

    private final SlurppIO splurppIO;
    private final SlurppIOInputsAutoLogged inputs = new SlurppIOInputsAutoLogged();

    public SlurppSubsystem(SlurppIO splurppIO) {
        super();
        this.splurppIO = splurppIO;
    }

    @Override
    public void periodic() {
        splurppIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Slurpp", inputs);
    }

    /** "Slurpp" up a game piece. */
    public void slurpp(double percentOutput) {
        splurppIO.setPercentOutput(percentOutput);
    }
    
    public void stop() {
        splurppIO.stop();
    }
}
