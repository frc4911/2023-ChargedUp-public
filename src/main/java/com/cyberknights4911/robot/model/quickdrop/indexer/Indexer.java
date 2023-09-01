package com.cyberknights4911.robot.model.quickdrop.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Indexer", inputs);
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
