package com.cyberknights4911.robot.model.deadeye;

import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.subsystems.drive.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public final class DeadEyeConstants {

    private DeadEyeConstants() {}
    
    private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 0;
    private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 0;
    private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 0;
    private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 0;

    private static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
        CotsFalconSwerveConstants.SDSMK2(
            CotsFalconSwerveConstants.DriveGearRatios.SDSMK2
        );

    public static final SwerveModuleConstants FRONT_LEFT = 
        new SwerveModuleConstants(
            0,
            DeadEyePorts.Drive.FRONT_LEFT_DRIVE,
            DeadEyePorts.Drive.FRONT_LEFT_STEER,
            DeadEyePorts.Drive.FRONT_LEFT_CANCODER,
            Rotation2d.fromDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants FRONT_RIGHT = 
        new SwerveModuleConstants(
            1,
            DeadEyePorts.Drive.FRONT_RIGHT_DRIVE,
            DeadEyePorts.Drive.FRONT_RIGHT_STEER,
            DeadEyePorts.Drive.FRONT_RIGHT_CANCODER,
            Rotation2d.fromDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_LEFT = 
        new SwerveModuleConstants(
            2,
            DeadEyePorts.Drive.BACK_LEFT_DRIVE,
            DeadEyePorts.Drive.BACK_LEFT_STEER,
            DeadEyePorts.Drive.BACK_LEFT_CANCODER,
            Rotation2d.fromDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);

    public static final SwerveModuleConstants BACK_RIGHT = 
        new SwerveModuleConstants(
            3,
            DeadEyePorts.Drive.BACK_RIGHT_DRIVE,
            DeadEyePorts.Drive.BACK_RIGHT_STEER,
            DeadEyePorts.Drive.BACK_RIGHT_CANCODER,
            Rotation2d.fromDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES),
            PHYSICAL_SWERVE_MODULE);
    
}
