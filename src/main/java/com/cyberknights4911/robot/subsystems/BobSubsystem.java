package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem for controlling the balancer (Bob).
 */
public final class BobSubsystem implements Subsystem {
    
    private final Solenoid mBobSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.BOB_SOLENOID_PORT);

    private boolean extended = false;

    public void setExtended (boolean extended) {

    mBobSolenoid.set(extended);

    }
}