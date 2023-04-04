package com.cyberknights4911.robot.config;

import com.cyberknights4911.robot.constants.Constants;

public interface RobotConfiguration {
    SwerveConfiguration getSwerveConfiguration();

    SwerveModuleConfiguration getFrontRightModuleConstants();

    SwerveModuleConfiguration getFrontLeftModuleConstants();

    SwerveModuleConfiguration getBackLeftModuleConstants();

    SwerveModuleConfiguration getBackRightModuleConstants();

    static RobotConfiguration getRobotConfiguration(String robotName) {
        switch (robotName) {
            // case Constants.kJuniorName:
            //     return new Junior();
            // case Constants.kDeadEyeName:
            //     return new DeadEye();
            case Constants.ROBOT_NAME_2023:
            default:
                return new Robot2023();
        }
    }    
}
