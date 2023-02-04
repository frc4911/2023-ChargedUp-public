package com.cyberknights4911.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling climbing
 */
public final class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO climberIO) {
        super();
        this.climberIO = climberIO;
    }
        
    @Override
    public void periodic() {
        climberIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Climber", inputs);
    }

    public void setExtended(boolean extended) {
        climberIO.setExtended(extended);
    }
}
