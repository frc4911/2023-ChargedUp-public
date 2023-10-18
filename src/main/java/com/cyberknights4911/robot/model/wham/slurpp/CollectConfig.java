package com.cyberknights4911.robot.model.wham.slurpp;

import com.cyberknights4911.robot.model.wham.WhamConstants;

public final class CollectConfig {
    private CollectConfig() {}

    public enum GamePiece {
        CONE, CUBE
    }

    public enum CollectSide {
        FRONT, BACK
    }

    private static final double FORWARD_MULTIPLIER = 1.0;
    private static final double BACKWARD_MULTIPLIER = -1.0;

    /**
     * Gets the appropriate speed for collecting the game piece.
     * @return the motor output percent
     */
    public static double collectSpeed(GamePiece gamePiece, CollectSide collectSide) {
        double baseSpeed = baseCollectScoreSpeed(gamePiece);
        switch(gamePiece) {
            case CONE: {
                switch(collectSide) {
                    case FRONT: return FORWARD_MULTIPLIER * baseSpeed;
                    case BACK: return BACKWARD_MULTIPLIER * baseSpeed;
                }
            }
            case CUBE: {
                switch(collectSide) {
                    case FRONT: return BACKWARD_MULTIPLIER * baseSpeed;
                    case BACK: return FORWARD_MULTIPLIER * baseSpeed;
                }
            }
        }
        // should never happen
        return baseSpeed;
    }

    /**
     * Gets the appropriate speed for scoring the game piece. Should be same magnitute as collect
     * speed, but opposite direction.
     * @return the motor output percent
     */
    public static double scoreSpeed(GamePiece gamePiece, CollectSide collectSide) {
        double baseSpeed = baseCollectScoreSpeed(gamePiece);
        switch(gamePiece) {
            case CONE: {
                switch(collectSide) {
                    case FRONT: return BACKWARD_MULTIPLIER * baseSpeed;
                    case BACK: return FORWARD_MULTIPLIER * baseSpeed;
                }
            }
            case CUBE: {
                switch(collectSide) {
                    case FRONT: return FORWARD_MULTIPLIER * baseSpeed;
                    case BACK: return BACKWARD_MULTIPLIER * baseSpeed;
                }
            }
        }
        // should never happen
        return baseSpeed;
    }

    /**
     * Gets the appropriate speed for retaining the game piece. Should be same direction as
     * collect speed, but weaker
     * @return the motor output percent
     */
    public static double retainSpeed(GamePiece gamePiece, CollectSide collectSide) {
        double baseSpeed = baseRetainSpeed(gamePiece);
        switch(gamePiece) {
            case CONE: {
                switch(collectSide) {
                    case FRONT: return FORWARD_MULTIPLIER * baseSpeed;
                    case BACK: return BACKWARD_MULTIPLIER * baseSpeed;
                }
            }
            case CUBE: {
                switch(collectSide) {
                    case FRONT: return BACKWARD_MULTIPLIER * baseSpeed;
                    case BACK: return FORWARD_MULTIPLIER * baseSpeed;
                }
            }
        }
        // should never happen
        return baseSpeed;
    }

    private static double baseCollectScoreSpeed(GamePiece gamePiece) {
        double speed = 0;
        switch(gamePiece) {
            case CONE: speed = WhamConstants.Slurpp.SLURPP_FAST_PERCENT.getValue();
            case CUBE: speed = WhamConstants.Slurpp.SLURPP_SLOW_PERCENT.getValue();
        }
        return speed;
    }

    private static double baseRetainSpeed(GamePiece gamePiece) {
        double speed = 0;
        switch(gamePiece) {
            case CONE: speed = WhamConstants.Slurpp.SLURPP_RETAIN_STRONG_PERCENT.getValue();
            case CUBE: speed = WhamConstants.Slurpp.SLURPP_RETAIN_WEAK_PERCENT.getValue();
        }
        return speed;
    }
}
