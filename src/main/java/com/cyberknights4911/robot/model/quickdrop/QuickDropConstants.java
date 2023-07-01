package com.cyberknights4911.robot.model.quickdrop;

import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.subsystems.drive.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public final class QuickDropConstants {

    private QuickDropConstants() {}

    public static final class Drive {
        private Drive() {}
        
        private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 330;
        private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 182.5;
        private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 262.7;
        private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 64.5;

        private static final CotsFalconSwerveConstants PHYSICAL_SWERVE_MODULE =
            CotsFalconSwerveConstants.SDSMK4i(
                CotsFalconSwerveConstants.DriveGearRatios.SDSMK4i_L1
            );

        public static final SwerveModuleConstants FRONT_LEFT = 
            new SwerveModuleConstants(
                QuickDropPorts.Drive.FRONT_LEFT_DRIVE,
                QuickDropPorts.Drive.FRONT_LEFT_STEER,
                QuickDropPorts.Drive.FRONT_LEFT_CANCODER,
                Rotation2d.fromDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants FRONT_RIGHT = 
            new SwerveModuleConstants(
                QuickDropPorts.Drive.FRONT_RIGHT_DRIVE,
                QuickDropPorts.Drive.FRONT_RIGHT_STEER,
                QuickDropPorts.Drive.FRONT_RIGHT_CANCODER,
                Rotation2d.fromDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants BACK_LEFT = 
            new SwerveModuleConstants(
                QuickDropPorts.Drive.BACK_LEFT_DRIVE,
                QuickDropPorts.Drive.BACK_LEFT_STEER,
                QuickDropPorts.Drive.BACK_LEFT_CANCODER,
                Rotation2d.fromDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES));

        public static final SwerveModuleConstants BACK_RIGHT = 
            new SwerveModuleConstants(
                QuickDropPorts.Drive.BACK_RIGHT_DRIVE,
                QuickDropPorts.Drive.BACK_RIGHT_STEER,
                QuickDropPorts.Drive.BACK_RIGHT_CANCODER,
                Rotation2d.fromDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES));
    }
    
}
