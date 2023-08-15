package com.cyberknights4911.robot.model.wham.climber;

import com.cyberknights4911.robot.model.wham.WhamPorts.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public final class ClimberIOReal implements ClimberIO, AutoCloseable {
    private final Solenoid solenoid;

    public ClimberIOReal() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Climber.SOLENOID);
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
