package com.cyberknights4911.robot.subsystems.climber;

import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem for controlling climbing
 */
public final class ClimberSubsystem implements Subsystem, AutoCloseable {
    private final Solenoid mClimbSolenoid;

    public ClimberSubsystem() {
        mClimbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.CLIMB_SOLENOID_PORT);
        mClimbSolenoid.set(false);
    }

    public void setExtended(boolean extended) {

        mClimbSolenoid.set(extended);

    }

    @Override
    public void close() throws Exception {
        mClimbSolenoid.close();
    }
}
