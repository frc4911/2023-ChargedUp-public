package com.cyberknights4911.robot.model.wham.slurpp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the Slurpp, which is responsible for taking in and releasing game
 * pieces.
 */
public final class SlurppSubsystem extends SubsystemBase {

    private final SlurppIO slurppIO;
    private final SlurppIOInputsAutoLogged inputs = new SlurppIOInputsAutoLogged();

    private CollectConfig.GamePiece gamePiece = CollectConfig.GamePiece.CONE;
    private CollectConfig.CollectSide collectSide = CollectConfig.CollectSide.FRONT;

    public SlurppSubsystem(SlurppIO splurppIO) {
        super();
        this.slurppIO = splurppIO;
    }

    @Override
    public void periodic() {
        slurppIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Slurpp", inputs);
    }

    public void setGamePiece(CollectConfig.GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public void setCollectSide(CollectConfig.CollectSide collectSide) {
        this.collectSide = collectSide;
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

    public CommandBase createCollectCommand() {
        return Commands.runOnce(() -> {
            slurpp(CollectConfig.collectSpeed(gamePiece, collectSide));
        }, this);
    }

    public CommandBase createScoreCommand() {
        return Commands.runOnce(() -> {
            slurpp(CollectConfig.scoreSpeed(gamePiece, collectSide));
        }, this);
    }

    public CommandBase createRetainCommand() {
        return Commands.runOnce(() -> {
            slurpp(CollectConfig.retainSpeed(gamePiece, collectSide));
        }, this);
    }
    
    public CommandBase createStopCommand() {
        return Commands.runOnce(() -> {
            stop();
        }, this);
    }
}
