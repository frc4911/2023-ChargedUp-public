package com.cyberknights4911.robot.subsystems.drive.v2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.util.Conversions;

public final class SwerveIOReal implements SwerveIO {
    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;

    private final SwerveModuleConstants swerveModuleConstants;

    public SwerveIOReal(
        SwerveModuleConstants swerveModuleConstants
    ) {
        this.swerveModuleConstants = swerveModuleConstants;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(swerveModuleConstants.getCanCoderId());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(swerveModuleConstants.getAngleMotorId());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(swerveModuleConstants.getDriveMotorId());
        configDriveMotor();
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveModuleConstants.getCtreConfigs().swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(swerveModuleConstants.getCtreConfigs().swerveAngleFXConfig);
        angleMotor.setInverted(swerveModuleConstants.getPhysicalSwerveModule().angleMotorInvert);
        angleMotor.setNeutralMode(Swerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(swerveModuleConstants.getCtreConfigs().swerveDriveFXConfig);
        driveMotor.setInverted(swerveModuleConstants.getPhysicalSwerveModule().driveMotorInvert);
        driveMotor.setNeutralMode(Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }
    
    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        // TODO: populate inputs
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
            getAngleEncoderDegrees() - swerveModuleConstants.getAngleOffset().getDegrees(),
            swerveModuleConstants.getPhysicalSwerveModule().angleGearRatio
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
