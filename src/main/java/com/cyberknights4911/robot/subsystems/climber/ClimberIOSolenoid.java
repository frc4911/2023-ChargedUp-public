package com.cyberknights4911.robot.subsystems.climber;

import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public final class ClimberIOSolenoid implements ClimberIO, AutoCloseable {
    private final Solenoid solenoid;

    public ClimberIOSolenoid() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.CLIMB_SOLENOID_PORT);
        solenoid.set(false);
    }
    
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.extended = solenoid.get();
    }

    @Override
    public void setExtended(boolean extended) {
        solenoid.set(extended);
    }

    @Override
    public void close() throws Exception {
        solenoid.close();
    }
}
