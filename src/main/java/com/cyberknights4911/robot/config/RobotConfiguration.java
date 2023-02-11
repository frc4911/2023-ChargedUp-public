package com.cyberknights4911.robot.config;

import com.cyberknights4911.robot.constants.Constants;

public interface RobotConfiguration {
    public SwerveConfiguration getSwerveConfiguration();

    public SwerveModuleConfiguration getFrontRightModuleConstants();

    public SwerveModuleConfiguration getFrontLeftModuleConstants();

    public SwerveModuleConfiguration getBackLeftModuleConstants();

    public SwerveModuleConfiguration getBackRightModuleConstants();

    public static RobotConfiguration getRobotConfiguration(String robotName) {
        switch (robotName) {
            // case Constants.kJuniorName:
            //     return new Junior();
            // case Constants.kDeadEyeName:
            //     return new DeadEye();
            case Constants.ROBOT_NAME_2022:
            default:
                return new Robot2022();
        }
    }    
}
