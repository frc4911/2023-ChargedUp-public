package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.subsystems.arm.ArmIO;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.bob.BobIO;
import com.cyberknights4911.robot.subsystems.bob.BobSubsystem;
import com.cyberknights4911.robot.subsystems.climber.ClimberIO;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodIO;
import com.cyberknights4911.robot.subsystems.hood.HoodIOReal;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppIO;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppSubsystem;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Subsystem creation manager. This ensures that the correct IO layer is passed to the constructor
 * according to the environment.
 */
public final class Subsystems {
    private final ClimberSubsystem climberSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final ArmSubsystem armSubsystem ;
    private final SwerveSubsystem swerveSubsystem;
    private final BobSubsystem bobSubsystem;
    private final HoodSubsystem hoodSubsystem;

    public Subsystems() {
        if (RobotBase.isReal()) {
            // TODO swap out these with real implementations once the hardware exists
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystemCurrent();
            bobSubsystem = new BobSubsystem(new BobIO() {});
            hoodSubsystem = new HoodSubsystem(new HoodIOReal());
        } else {
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystemCurrent();
            bobSubsystem = new BobSubsystem(new BobIO() {});
            hoodSubsystem = new HoodSubsystem(new HoodIO() {});
        }
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
