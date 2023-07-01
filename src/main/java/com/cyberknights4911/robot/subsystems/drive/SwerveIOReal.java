package com.cyberknights4911.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.util.Conversions;

import libraries.cyberlib.drivers.TalonFXFactory;

public final class SwerveIOReal implements SwerveIO {
    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;

    private final SwerveModuleConstants swerveModuleConstants;
    private final CotsFalconSwerveConstants cotsFalconSwerveConstants;
    private final CtreConfigs ctreConfigs;

    public SwerveIOReal(
        SwerveModuleConstants swerveModuleConstants,
        CotsFalconSwerveConstants cotsFalconSwerveConstants
    ) {
        this.swerveModuleConstants = swerveModuleConstants;
        this.cotsFalconSwerveConstants = cotsFalconSwerveConstants;

        this.ctreConfigs = new CtreConfigs(cotsFalconSwerveConstants);
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(swerveModuleConstants.getCanCoderId());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = TalonFXFactory.createTalon(swerveModuleConstants.getAngleMotorId());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = TalonFXFactory.createTalon(swerveModuleConstants.getDriveMotorId());
        configDriveMotor();
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        // ctreConfigs.swerveCanCoderConfig.magnetOffsetDegrees = swerveModuleConstants.getAngleOffset().getDegrees();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(cotsFalconSwerveConstants.angleMotorInvert);
        angleMotor.setNeutralMode(Swerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(cotsFalconSwerveConstants.driveMotorInvert);
        driveMotor.setNeutralMode(Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }
    
    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.driveVelocityRpm = Conversions.falconToMPS(
            getDriveSensorVelocity(),
            cotsFalconSwerveConstants.wheelCircumference,
            cotsFalconSwerveConstants.driveGearRatio
        );
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = driveMotor.getSupplyCurrent();
        inputs.driveTempCelcius = driveMotor.getTemperature();
        
        inputs.turnPositionDeg = 0;
        inputs.turnAppliedVolts = angleMotor.getMotorOutputVoltage();
        inputs.turnCurrentAmps = angleMotor.getSupplyCurrent();
        inputs.turnTempCelcius = angleMotor.getTemperature();
        
        inputs.turnAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
            getAngleEncoderDegrees() - swerveModuleConstants.getAngleOffset().getDegrees(),
            cotsFalconSwerveConstants.angleGearRatio
        );
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }
  
    @Override
    public void setAngle(ControlMode mode, double outputValue) {
        angleMotor.set(mode, outputValue);
    }

    @Override
    public void setDrive(ControlMode mode, double outputValue) {
        driveMotor.set(mode, outputValue);
    }
    
    @Override
    public void setDrive(
        ControlMode mode, double outputValue, DemandType demandType, double demandValue
    ) {
        driveMotor.set(mode, outputValue, demandType, demandValue);
    }
    
    @Override
    public double getDriveSensorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    @Override
    public double getAngleSensorPosition() {
        return angleMotor.getSelectedSensorPosition();
    }
    
    @Override
    public double getDriveSensorVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    @Override
    public double getAngleEncoderDegrees() {
        return angleEncoder.getAbsolutePosition();
    }
}
