package com.cyberknights4911.robot.subsystems.slurpp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the Slurpp, which is responsible for taking in and releasing game
 * pieces.
 */
public final class SlurppSubsystem extends SubsystemBase {

    private final SplurppIO splurppIO;
    private final SplurppIOInputsAutoLogged inputs = new SplurppIOInputsAutoLogged();

    public SlurppSubsystem(SplurppIO splurppIO) {
        super();
        this.splurppIO = splurppIO;
    }
        
    @Override
    public void periodic() {
        splurppIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Slurpp", inputs);
    }

    /** "Slurpp" up a game piece. */
    public void slurpp() {
        splurppIO.setPercentOutput(0.3);
    }

    /** "Spit" out a game piece. */
    public void spit() {
        splurppIO.setPercentOutput(-0.3);
    }
    
    public void stop() {
        splurppIO.stop();
    }
}
