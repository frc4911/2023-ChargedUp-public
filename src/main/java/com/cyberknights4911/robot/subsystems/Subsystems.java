package com.cyberknights4911.robot.subsystems;

import com.cyberknights4911.robot.config.RobotConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.arm.ArmIO;
import com.cyberknights4911.robot.subsystems.arm.ArmIOReal;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.bob.BobIO;
import com.cyberknights4911.robot.subsystems.bob.BobSubsystem;
import com.cyberknights4911.robot.subsystems.climber.ClimberIO;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;
import com.cyberknights4911.robot.subsystems.drive.GyroIO;
import com.cyberknights4911.robot.subsystems.drive.GyroIOReal;
import com.cyberknights4911.robot.subsystems.drive.SwerveIO;
import com.cyberknights4911.robot.subsystems.drive.SwerveIOReal;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodIO;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem;
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

    public Subsystems() {
        RobotConfiguration configuration =  RobotConfiguration.getRobotConfiguration(Constants.ROBOT_NAME_2023);
        if (RobotBase.isReal()) {
            // TODO swap out these with real implementations once the hardware exists
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIOReal());
            armSubsystem = new ArmSubsystem(new ArmIOReal());
            swerveSubsystem = new SwerveSubsystemCurrent(
                configuration,
                new GyroIOReal(),
                new SwerveIOReal(configuration.getFrontLeftModuleConstants()),
                new SwerveIOReal(configuration.getFrontRightModuleConstants()),
                new SwerveIOReal(configuration.getBackLeftModuleConstants()),
                new SwerveIOReal(configuration.getBackRightModuleConstants())
            );
            bobSubsystem = new BobSubsystem(new BobIO() {});
        } else {
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
            slurppSubsystem = new SlurppSubsystem(new SlurppIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            swerveSubsystem = new SwerveSubsystemCurrent(
                configuration,
                new GyroIO() {},
                new SwerveIO() {},
                new SwerveIO() {},
                new SwerveIO() {},
                new SwerveIO() {}
            );
            bobSubsystem = new BobSubsystem(new BobIO() {});
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
}
