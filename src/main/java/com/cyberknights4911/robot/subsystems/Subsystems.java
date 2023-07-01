package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.subsystems.arm.ArmIO;
import com.cyberknights4911.robot.subsystems.arm.ArmIOReal;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.camera.CollectorCamera;
import com.cyberknights4911.robot.subsystems.climber.ClimberIO;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.collector.Collector;
import com.cyberknights4911.robot.subsystems.collector.CollectorIO;
import com.cyberknights4911.robot.subsystems.collector.CollectorIOReal;
import com.cyberknights4911.robot.subsystems.drive.GyroIO;
import com.cyberknights4911.robot.subsystems.drive.GyroIOReal;
import com.cyberknights4911.robot.subsystems.drive.GyroIORealPigeon1;
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
    private final Collector collector;
    private final CollectorCamera camera;

    public Subsystems() {
        if (RobotBase.isReal()) {
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystem(
                new GyroIORealPigeon1(),
                new SwerveIOReal(SwerveModuleConstants.FRONT_LEFT),
                new SwerveIOReal(SwerveModuleConstants.FRONT_RIGHT),
                new SwerveIOReal(SwerveModuleConstants.BACK_LEFT),
                new SwerveIOReal(SwerveModuleConstants.BACK_RIGHT)
            );
            collector = new Collector(new CollectorIO() {});
            camera = new CollectorCamera();
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
            collector = new Collector(new CollectorIO() {});
            camera = new CollectorCamera();
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

    public Collector getCollector() {
        return collector;
    }

    public CollectorCamera getCamera() {
        return camera;
    }
}
