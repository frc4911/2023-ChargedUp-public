package com.cyberknights4911.robot.model.wham.slurpp;

import com.cyberknights4911.robot.model.wham.slurpp.CollectConfig.CollectSide;
import com.cyberknights4911.robot.model.wham.slurpp.CollectConfig.GamePiece;

public final class SlurppIOFake implements SlurppIO {

    @Override
    public void updateInputs(SlurppIOInputs inputs) {}

    @Override
    public void setPercentOutput(double percentOutput) {}

    @Override
    public void stop() {}

    @Override
    public void setGamePiece(GamePiece gamePiece) {}

    @Override
    public void setCollectSide(CollectSide collectSide) {}

    @Override
    public GamePiece getGamePiece() {
        return null;
    }

    @Override
    public CollectSide getCollectSide() {
        return null;
    }
    
}
