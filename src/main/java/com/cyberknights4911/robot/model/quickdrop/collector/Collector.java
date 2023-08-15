package com.cyberknights4911.robot.model.quickdrop.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Collector extends SubsystemBase {

    private final CollectorIO collectorIO;

    public Collector(CollectorIO collectorIO) {
        this.collectorIO = collectorIO;
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
