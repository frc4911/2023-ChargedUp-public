package com.cyberknights4911.robot.subsystems.drive;

import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveModuleConstants {
    private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 330;
    private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 182.5;
    private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 262.7;
    private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 64.5;

    private static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
        CotsFalconSwerveConstants.SDSMK2(
            CotsFalconSwerveConstants.DriveGearRatios.SDSMK2
        );

    public static final SwerveModuleConstants FRONT_LEFT = 
        new SwerveModuleConstants(
            0,
            Ports.Drive.FRONT_LEFT_DRIVE,
            Ports.Drive.FRONT_LEFT_STEER,
            Ports.Drive.FRONT_LEFT_CANCODER,
            Rotation2d.fromDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants FRONT_RIGHT = 
        new SwerveModuleConstants(
            1,
            Ports.Drive.FRONT_RIGHT_DRIVE,
            Ports.Drive.FRONT_RIGHT_STEER,
            Ports.Drive.FRONT_RIGHT_CANCODER,
            Rotation2d.fromDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_LEFT = 
        new SwerveModuleConstants(
            2,
            Ports.Drive.BACK_LEFT_DRIVE,
            Ports.Drive.BACK_LEFT_STEER,
            Ports.Drive.BACK_LEFT_CANCODER,
            Rotation2d.fromDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_RIGHT = 
        new SwerveModuleConstants(
            3,
            Ports.Drive.BACK_RIGHT_DRIVE,
            Ports.Drive.BACK_RIGHT_STEER,
            Ports.Drive.BACK_RIGHT_CANCODER,
            Rotation2d.fromDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES),
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
