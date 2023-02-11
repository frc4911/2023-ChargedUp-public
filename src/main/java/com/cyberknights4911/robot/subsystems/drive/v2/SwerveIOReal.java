package com.cyberknights4911.robot.subsystems.drive.v2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.util.Conversions;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveIOReal implements SwerveIO {
    private final Rotation2d angleOffset;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;
    
    private final CotsFalconSwerveConstants physicalSwerveModule;
    private final CtreConfigs ctreConfigs;

    public SwerveIOReal(
        int moduleNumber,
        int driveMotorId,
        int angleMotorId,
        int canCoderId,
        Rotation2d angleOffset,
        CotsFalconSwerveConstants physicalSwerveModule,
        CtreConfigs ctreConfigs
    ) {
        this.angleOffset = angleOffset;
        this.physicalSwerveModule = physicalSwerveModule;
        this.ctreConfigs = ctreConfigs;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(canCoderId, Constants.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorId, Constants.CANIVORE_NAME);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorId, Constants.CANIVORE_NAME);
        configDriveMotor();
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(physicalSwerveModule.angleMotorInvert);
        angleMotor.setNeutralMode(Swerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(physicalSwerveModule.driveMotorInvert);
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
            getAngleEncoderDegrees() - angleOffset.getDegrees(),
            physicalSwerveModule.angleGearRatio
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
