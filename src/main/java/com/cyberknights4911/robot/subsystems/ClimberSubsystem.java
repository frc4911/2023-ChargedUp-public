package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem for controlling climbing
 */
public final class ClimberSubsystem implements Subsystem {
    private final Solenoid mClimbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.CLIMB_SOLENOID_PORT);

    private boolean extended = false;

    public void setExtended (boolean extended) {

        mClimbSolenoid.set(extended);

    }
    @Override
    public void periodic () {

        //TODO(SuperDuperGreen) add periodic code here
    }
}
