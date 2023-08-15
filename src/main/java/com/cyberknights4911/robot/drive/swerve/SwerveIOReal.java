package com.cyberknights4911.robot.drive.swerve;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.util.Conversions;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class SwerveIOReal implements SwerveIO {
    private final WPI_TalonFX angleMotor;
    private final WPI_TalonFX driveMotor;
    private final WPI_CANCoder angleEncoder;

    private final CtreError ctreError;
    private final SwerveModuleConstants swerveModuleConstants;
    private final SwerveDriveConstants swerveDriveConstants;
    private final CotsFalconSwerveConstants cotsConstants;
    private final CtreConfigs ctreConfigs;

    public static Supplier<SwerveIO> getSupplier(
        TalonFXFactory talonFXFactory,
        CANCoderFactory canCoderFactory,
        CtreError ctreError,
        SwerveModuleConstants swerveModuleConstants,
        SwerveDriveConstants swerveDriveConstants,
        CotsFalconSwerveConstants cotsConstants
    ) {
        return () -> {
            return new SwerveIOReal(talonFXFactory, canCoderFactory, ctreError, swerveModuleConstants, swerveDriveConstants, cotsConstants);
        };
    }

    private SwerveIOReal(
        TalonFXFactory talonFXFactory,
        CANCoderFactory canCoderFactory,
        CtreError ctreError,
        SwerveModuleConstants swerveModuleConstants,
        SwerveDriveConstants swerveDriveConstants,
        CotsFalconSwerveConstants cotsConstants
    ) {
        this.ctreError = ctreError;
        this.swerveModuleConstants = swerveModuleConstants;
        this.swerveDriveConstants = swerveDriveConstants;
        this.cotsConstants = cotsConstants;

        this.ctreConfigs = new CtreConfigs(cotsConstants, swerveDriveConstants);
        
        /* Angle Encoder Config */
        angleEncoder = canCoderFactory.create(swerveModuleConstants.canCoderId());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = talonFXFactory.createTalon(swerveModuleConstants.angleMotorId());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = talonFXFactory.createTalon(swerveModuleConstants.driveMotorId());
        configDriveMotor();
    }

    private void configAngleEncoder() {
        ctreError.checkError(angleEncoder.configFactoryDefault(ctreError.canTimeoutMs()));
        // ctreConfigs.swerveCanCoderConfig.magnetOffsetDegrees = swerveModuleConstants.getAngleOffset().getDegrees();
        ctreError.checkError(angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig, ctreError.canTimeoutMs()));
    }

    private void configAngleMotor() {
        ctreError.checkError(angleMotor.configFactoryDefault(ctreError.canTimeoutMs()));
        ctreError.checkError(angleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig, ctreError.canTimeoutMs()));
        angleMotor.setInverted(cotsConstants.angleMotorInvert());
        angleMotor.setNeutralMode(swerveDriveConstants.angleNeutralMode());
        resetToAbsolute();
    }

    private void configDriveMotor() {
        ctreError.checkError(driveMotor.configFactoryDefault(ctreError.canTimeoutMs()));
        ctreError.checkError(driveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig, ctreError.canTimeoutMs()));
        driveMotor.setInverted(cotsConstants.driveMotorInvert());
        driveMotor.setNeutralMode(swerveDriveConstants.driveNeutralMode());
        driveMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.driveVelocityRpm = Conversions.falconToMPS(
            getDriveSensorVelocity(),
            cotsConstants.wheelCircumference(),
            cotsConstants.driveGearRatio()
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
            getAngleEncoderDegrees() - swerveModuleConstants.angleOffset().getDegrees(),
            cotsConstants.angleGearRatio()
        );
        ctreError.checkError(angleMotor.setSelectedSensorPosition(absolutePosition, /* pidIdx = */ 0, ctreError.canTimeoutMs()));
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
