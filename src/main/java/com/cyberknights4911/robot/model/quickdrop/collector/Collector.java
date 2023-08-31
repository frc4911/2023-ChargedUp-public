package com.cyberknights4911.robot.model.quickdrop.collector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Collector extends SubsystemBase {

    private final CollectorIO collectorIO;
    private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

    public Collector(CollectorIO collectorIO) {
        this.collectorIO = collectorIO;
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Collector", inputs);
    }

    public void run(double percentOutput) {
        collectorIO.setPercentOutput(percentOutput);
    }

    public void stop() {
        collectorIO.stop();
    }

    public void extend() {
        collectorIO.extend();
    }
    
    public void retract() {
        collectorIO.retract();
    }
}
