package com.cyberknights4911.robot.drive.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;

public final class CtreConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CtreConfigs(
        CotsFalconSwerveConstants cotsConstants,
        SwerveDriveConstants swerveDriveConstants
    ) {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            swerveDriveConstants.angleEnableCurrentLimit(),
            swerveDriveConstants.angleContinuousCurrentLimit(),
            swerveDriveConstants.anglePeakCurrentLimit(),
            swerveDriveConstants.anglePeakCurrentDuration());

        swerveAngleFXConfig.slot0.kP = cotsConstants.angleKP();
        swerveAngleFXConfig.slot0.kI = cotsConstants.angleKI();
        swerveAngleFXConfig.slot0.kD = cotsConstants.angleKD();
        swerveAngleFXConfig.slot0.kF = cotsConstants.angleKF();
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            swerveDriveConstants.driveEnableCurrentLimit(),
            swerveDriveConstants.driveContinuousCurrentLimit(),
            swerveDriveConstants.drivePeakCurrentLimit(),
            swerveDriveConstants.drivePeakCurrentDuration());

        swerveDriveFXConfig.slot0.kP = swerveDriveConstants.driveProportionalGain();
        swerveDriveFXConfig.slot0.kI = swerveDriveConstants.driveIntegralGain();
        swerveDriveFXConfig.slot0.kD = swerveDriveConstants.driveDerivativeGain();
        swerveDriveFXConfig.slot0.kF = swerveDriveConstants.driveFeedForwardGain();
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = swerveDriveConstants.openloopRamp();
        swerveDriveFXConfig.closedloopRamp = swerveDriveConstants.closedloopRamp();
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = cotsConstants.canCoderInvert();
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
