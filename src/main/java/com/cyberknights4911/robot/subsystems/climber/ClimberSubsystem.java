package com.cyberknights4911.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem for controlling climbing
 */
public final class ClimberSubsystem implements Subsystem {
    private final ClimberIO climberIO;

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public void setExtended(boolean extended) {
        climberIO.setExtended(extended);
    }
}
