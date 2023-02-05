package com.cyberknights4911.robot.subsystems.drive.v2;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;

public final class CtreConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CtreConfigs(CotsFalconSwerveConstants physicalSwerveModule) {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Swerve.ANGLE_ENABLE_CURRENT_LIMIT, 
            Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Swerve.ANGLE_PEAK_CURRENT_LIMIT, 
            Swerve.ANGLE_PEAK_CURRENT_DURATION
        );

        swerveAngleFXConfig.slot0.kP = physicalSwerveModule.angleKP;
        swerveAngleFXConfig.slot0.kI = physicalSwerveModule.angleKI;
        swerveAngleFXConfig.slot0.kD = physicalSwerveModule.angleKD;
        swerveAngleFXConfig.slot0.kF = physicalSwerveModule.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Swerve.DRIVE_ENABLE_CURRENT_LIMIT, 
            Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Swerve.DRIVE_PEAK_CURRENT_LIMIT, 
            Swerve.DRIVE_PEAK_CURRENT_DURATION
        );

        swerveDriveFXConfig.slot0.kP = Swerve.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Swerve.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Swerve.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Swerve.DRIVE_KF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Swerve.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = physicalSwerveModule.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
