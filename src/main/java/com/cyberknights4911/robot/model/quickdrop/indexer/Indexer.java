package com.cyberknights4911.robot.model.quickdrop.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Indexer extends SubsystemBase {
    
    private final IndexerIO indexerIO;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public void run(double percentOutput) {
        indexerIO.setPercentOutput(percentOutput);
    }

    public void stop() {
        indexerIO.stop();
    }

    public boolean isEnterBlocked() {
        return indexerIO.isEnterBlocked();
    }

    public boolean isExitBlocked() {
        return indexerIO.isExitBlocked();
    }
}
