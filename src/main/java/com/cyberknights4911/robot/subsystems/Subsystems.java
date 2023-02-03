package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.bob.BobSubsystem;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppIOfalconFX;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppSubsystem;

/**
 * Subsystem instantiation manager. This ensures that the correct IO layer is passed to the constructor.
 */
public final class Subsystems {
    private final ClimberSubsystem climberSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final ArmSubsystem armSubsystem ;
    private final SwerveSubsystem swerveSubsystem;
    private final BobSubsystem bobSubsystem;
    private final HoodSubsystem hoodSubsystem;

    public Subsystems() {
        // TODO(riley) switch these deps out with sim versions when not in real mode
        climberSubsystem = new ClimberSubsystem();
        slurppSubsystem = new SlurppSubsystem(new SlurppIOfalconFX());
        armSubsystem = new ArmSubsystem();
        swerveSubsystem = new SwerveSubsystem();
        bobSubsystem = new BobSubsystem();
        hoodSubsystem = new HoodSubsystem();
    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public SlurppSubsystem getSlurppSubsystem() {
        return slurppSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public BobSubsystem getBobSubsystem() {
        return bobSubsystem;
    }

    public HoodSubsystem getHoodSubsystem() {
        return hoodSubsystem;
    }
}
