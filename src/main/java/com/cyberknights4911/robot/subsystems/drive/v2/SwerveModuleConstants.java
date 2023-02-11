package com.cyberknights4911.robot.subsystems.drive.v2;

import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveModuleConstants {
    private static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
        CotsFalconSwerveConstants.SDSMK4i(
            CotsFalconSwerveConstants.DriveGearRatios.SDSMK4i_L2
        );

    public static final SwerveModuleConstants FRONT_LEFT = 
        new SwerveModuleConstants(
            0,
            Ports.ROBOT_2022_FRONT_LEFT_DRIVE,
            Ports.ROBOT_2022_FRONT_LEFT_STEER,
            Ports.ROBOT_2022_FRONT_LEFT_CANCODER,
            Rotation2d.fromDegrees(150.38),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants FRONT_RIGHT = 
        new SwerveModuleConstants(
            1,
            Ports.ROBOT_2022_FRONT_RIGHT_DRIVE,
            Ports.ROBOT_2022_FRONT_RIGHT_STEER,
            Ports.ROBOT_2022_FRONT_RIGHT_CANCODER,
            Rotation2d.fromDegrees(2.29),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_LEFT = 
        new SwerveModuleConstants(
            2,
            Ports.ROBOT_2022_BACK_LEFT_DRIVE,
            Ports.ROBOT_2022_BACK_LEFT_STEER,
            Ports.ROBOT_2022_BACK_LEFT_CANCODER,
            Rotation2d.fromDegrees(83.23),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_RIGHT = 
        new SwerveModuleConstants(
            3,
            Ports.ROBOT_2022_BACK_RIGHT_DRIVE,
            Ports.ROBOT_2022_BACK_RIGHT_STEER,
            Ports.ROBOT_2022_BACK_RIGHT_CANCODER,
            Rotation2d.fromDegrees(244.07),
            PHYSICAL_SWERVE_MODULE);

    private final int moduleNumber;
    private final int driveMotorId;
    private final int angleMotorId;
    private final int canCoderId;
    private final Rotation2d angleOffset;
    private final CotsFalconSwerveConstants physicalSwerveModule;
    private final CtreConfigs ctreConfigs;

    private SwerveModuleConstants(
        int moduleNumber,
        int driveMotorId,
        int angleMotorId,
        int canCoderId,
        Rotation2d angleOffset,
        CotsFalconSwerveConstants physicalSwerveModule
    ) {
        this.moduleNumber = moduleNumber;
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
        this.physicalSwerveModule = physicalSwerveModule;
        this.ctreConfigs = new CtreConfigs(physicalSwerveModule);
    }
    
    public int getModuleNumber() {
        return moduleNumber;
    }

    public int getDriveMotorId() {
        return driveMotorId;
    }

    public int getAngleMotorId() {
        return angleMotorId;
    }

    public int getCanCoderId() {
        return canCoderId;
    }

    public Rotation2d getAngleOffset() {
        return angleOffset;
    }

    public CotsFalconSwerveConstants getPhysicalSwerveModule() {
        return physicalSwerveModule;
    }

    public CtreConfigs getCtreConfigs() {
        return ctreConfigs;
    }
}
