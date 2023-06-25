package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.subsystems.arm.ArmIO;
import com.cyberknights4911.robot.subsystems.arm.ArmIOReal;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.bob.BobIO;
import com.cyberknights4911.robot.subsystems.bob.BobSubsystem;
import com.cyberknights4911.robot.subsystems.climber.ClimberIO;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.collector.Collector;
import com.cyberknights4911.robot.subsystems.collector.CollectorIO;
import com.cyberknights4911.robot.subsystems.collector.CollectorIOReal;
import com.cyberknights4911.robot.subsystems.drive.GyroIO;
import com.cyberknights4911.robot.subsystems.drive.GyroIOReal;
import com.cyberknights4911.robot.subsystems.drive.SwerveIO;
import com.cyberknights4911.robot.subsystems.drive.SwerveIOReal;
import com.cyberknights4911.robot.subsystems.drive.SwerveModuleConstants;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppIO;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppIOReal;
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
    private final Collector collector;

    public Subsystems() {
        if (RobotBase.isReal()) {
            // TODO swap out these with real implementations once the hardware exists
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystem(
                new GyroIOReal(),
                new SwerveIOReal(SwerveModuleConstants.FRONT_LEFT),
                new SwerveIOReal(SwerveModuleConstants.FRONT_RIGHT),
                new SwerveIOReal(SwerveModuleConstants.BACK_LEFT),
                new SwerveIOReal(SwerveModuleConstants.BACK_RIGHT)
            );
            bobSubsystem = new BobSubsystem(new BobIO() {});
            collector = new Collector(new CollectorIOReal());
        } else {
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystem(
                new GyroIO() {},
                new SwerveIO() {},
                new SwerveIO() {},
                new SwerveIO() {},
                new SwerveIO() {}
            );
            bobSubsystem = new BobSubsystem(new BobIO() {});
            collector = new Collector(new CollectorIO() {});
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

    public Collector getCollector() {
        return collector;
    }
}
