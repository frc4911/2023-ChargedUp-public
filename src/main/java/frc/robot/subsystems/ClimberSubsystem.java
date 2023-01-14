package frc.robot.subsystems;

import java.time.Period;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Subsystem for controlling climbing
 */
public final class ClimberSubsystem implements Subsystem {
    private final Solenoid climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_SOLANOID_PORT);

    private boolean extended = false;

    public void setExtended (boolean extended) {

        this.extended = extended;

    }
    @Override
    public void periodic () {
        climbSolenoid.set(extended);
    }
}

