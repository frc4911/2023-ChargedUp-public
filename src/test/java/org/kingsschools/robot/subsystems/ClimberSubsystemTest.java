package org.kingsschools.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.cyberknights4911.robot.constants.Ports;
import com.cyberknights4911.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;;

public class ClimberSubsystemTest {

    private ClimberSubsystem climberSubsystem;
    private SolenoidSim solenoidSim;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);
        climberSubsystem = new ClimberSubsystem();
        // Keep args in sync with ClimberSubsystem.java
        solenoidSim = new SolenoidSim(PneumaticsModuleType.REVPH, Ports.CLIMB_SOLENOID_PORT);
    }

    @AfterEach
    public void tearDown() throws Exception {
        climberSubsystem.close();
    }

    @Test
    public void testInitialState() {
        assertFalse(solenoidSim.getOutput());
    }

    @Test
    public void testSetExtended() {
        climberSubsystem.setExtended(true);

        assertTrue(solenoidSim.getOutput());
    }
    
}