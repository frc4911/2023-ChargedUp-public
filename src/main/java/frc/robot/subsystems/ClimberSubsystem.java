package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Ports;

/*
 Subsystem for controlling climbing
 */
public final class ClimberSubsystem implements Subsystem {
    private final Solenoid mClimbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.CLIMB_SOLENOID_PORT);

    private boolean extended = false;

    public void setExtended (boolean extended) {

    mClimbSolenoid.set(extended);

    }
    @Override
    public void periodic () {
//heh
    }
}
