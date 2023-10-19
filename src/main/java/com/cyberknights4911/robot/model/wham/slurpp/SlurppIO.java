package com.cyberknights4911.robot.model.wham.slurpp;

import org.littletonrobotics.junction.AutoLog;

public interface SlurppIO {

    @AutoLog
    class SlurppIOInputs {
      public double positionDeg = 0.0;
      public double velocityRpm = 0.0;
      public double appliedVolts = 0.0;
      public double currentAmps = 0.0;
      public double tempCelcius = 0.0;
      public String gamePiece = "";
      public String collectSide = "";
    }

    /** Updates the set of loggable inputs. */
    void updateInputs(SlurppIOInputs inputs);

    /** Set motor percent output. */
    void setPercentOutput(double percentOutput);

    /** Stop the motor. */
    void stop();

    /** Hold current position. */
    void holdCurrentPosition();
    
    /**
     * Sets the game piece type that slurpp is dealing with. This, along with
     * {@link setCollectSide}, ensures that the driver/operator can always use the same
     * trigger/bumper for collect.
     */
    void setGamePiece(CollectConfig.GamePiece gamePiece);

    /**
     * Sets the collect side (front or back) of the robot. This, along with {@link setGamePiece},
     * ensures that the driver/operator can always use the same trigger/bumper for collect.
     */
    void setCollectSide(CollectConfig.CollectSide collectSide);

    /**
     * Gets the game piece type that slurpp is dealing with.
     */
    CollectConfig.GamePiece getGamePiece();

    /**
     * Gets the collect side (front or back) of the robot.
     */
    CollectConfig.CollectSide getCollectSide();
}
