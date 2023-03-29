package com.cyberknights4911.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.cyberknights4911.robot.config.SwerveModuleConfiguration;
import com.cyberknights4911.robot.constants.Constants;

import edu.wpi.first.math.util.Units;
import libraries.cheesylib.control.FramePeriodSwitch;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class SwerveIOReal implements SwerveIO {
    private static final double DRIVE_TICKS_PER_REV = 2048;
    private static final double TURN_TICKS_PER_REV = 2048;

    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final CANCoder encoder;
    private final SwerveModuleConfiguration swerveConfig;

    public SwerveIOReal(SwerveModuleConfiguration swerveConfig) {
        this.swerveConfig = swerveConfig;
        turnMotor = TalonFXFactory.createTalon(swerveConfig.steerMotorTalonId);
        driveMotor = TalonFXFactory.createTalon(swerveConfig.driveMotorTalonId);
        encoder = new CANCoder(swerveConfig.CANCoderId);
        
        configCancoder();
        configureMotors();
    }

    private void configCancoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = swerveConfig.CAN_CODER_SENSOR_INITIALIZATION_STRATEGY;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = swerveConfig.CANCoderOffsetDegrees;
        config.sensorDirection = false;

        encoder.configAllSettings(config, Constants.LONG_CAN_TIMEOUTS_MS);
    }

    /**
     * Configure motors based on current SwerveModuleConstants.
     */
    private void configureMotors() {
        encoder.configMagnetOffset(swerveConfig.CANCoderOffsetDegrees, 200);

        commonMotorConfig(driveMotor, "Drive");
        commonMotorConfig(turnMotor, "Steer");

        turnMotor.setInverted(swerveConfig.invertSteerMotor);
        turnMotor.configMotionAcceleration(0.9 * swerveConfig.steerTicksPerUnitVelocity * 0.25, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.configMotionCruiseVelocity(0.9 * swerveConfig.steerTicksPerUnitVelocity,Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.configVelocityMeasurementPeriod(swerveConfig.STEER_MOTOR_VELOCITY_MEASUREMENT_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.configVelocityMeasurementWindow(swerveConfig.STEER_MOTOR_VELOCITY_MEASUREMENT_WINDOW, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.selectProfileSlot(0, 0);

        // Slot 0 is for normal use (tuned for fx integrated encoder)
        turnMotor.config_kP(0, swerveConfig.steerMotorSlot0Kp, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.config_kI(0, swerveConfig.steerMotorSlot0Ki, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.config_kD(0, swerveConfig.steerMotorSlot0Kd, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.config_kF(0, swerveConfig.steerMotorSlot0Kf, Constants.LONG_CAN_TIMEOUTS_MS);
        turnMotor.config_IntegralZone(0, swerveConfig.STEER_MOTOR_SLOT_0_I_ZONE, Constants.LONG_CAN_TIMEOUTS_MS);
        
        driveMotor.setInverted(swerveConfig.invertDrive);
        driveMotor.configOpenloopRamp(0.3, Constants.LONG_CAN_TIMEOUTS_MS); // Increase if swerve acceleration is too fast

        FramePeriodSwitch.configStatorCurrentLimitPermanent(
            driveMotor,
            new StatorCurrentLimitConfiguration(true, 40, 0, 0));
    }

    private static void commonMotorConfig(TalonFX motor, String motorName) {
        System.out.println("configuring "+motorName+" motor");

        // The following commands are stored in nonVolatile ram in the motor
        // They are repeated on boot incase a motor needs to replaced quickly
        FramePeriodSwitch.configFactoryDefaultPermanent(motor);

        // the following commands are stored in nonVolatile ram but they are
        // no longer deemed necessary. Keeping around for a while in case they
        // need to be brought back
        // motor.configNeutralDeadband(.04, 100);
        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);
        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 100);

        // the following are volatile settings and must be run every power cycle
        FramePeriodSwitch.setFramePeriodsVolatile(motor); // set frame periods

        FramePeriodSwitch.setNeutralModeVolatile(motor, NeutralMode.Brake);
    }
    
    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.drivePositionDeg = Units.rotationsToDegrees(
            driveMotor.getSelectedSensorPosition() / DRIVE_TICKS_PER_REV / swerveConfig.driveReduction);
        inputs.driveVelocityRpm = 
            driveMotor.getSelectedSensorVelocity() * 10 / DRIVE_TICKS_PER_REV / swerveConfig.driveReduction;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = driveMotor.getSupplyCurrent();
        inputs.driveTempCelcius = driveMotor.getTemperature();
        
        inputs.turnPositionDeg = Units.rotationsToDegrees(
            turnMotor.getSelectedSensorPosition() / TURN_TICKS_PER_REV / swerveConfig.steerReduction);
        inputs.turnVelocityRpm =
            turnMotor.getSelectedSensorVelocity() * 10 / TURN_TICKS_PER_REV / swerveConfig.steerReduction;
        inputs.turnAppliedVolts = turnMotor.getMotorOutputVoltage();
        inputs.turnCurrentAmps = turnMotor.getSupplyCurrent();
        inputs.turnTempCelcius = turnMotor.getTemperature();
        
        inputs.turnAbsolutePositionDeg = encoder.getAbsolutePosition();
    }
  
    @Override
    public void setTurn(ControlMode mode, double outputValue) {
        turnMotor.set(mode, outputValue);
    }

    @Override
    public void setDrive(ControlMode mode, double outputValue) {
        driveMotor.set(mode, outputValue);
    }
    
    @Override
    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }
    
    @Override
    public double getTurnPosition() {
        return turnMotor.getSelectedSensorPosition();
    }
    
    @Override
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    @Override
    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity();
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }
  
    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }
    
    @Override
    public double getTurnSensorPosition() {
        return turnMotor.getSelectedSensorPosition();
    }
    
    @Override
    public void setTurnSensorPosition(double value) {
        turnMotor.setSelectedSensorPosition(value, 0, 0);
    }

    @Override
    public double getLastTimeStamp() {
        return encoder.getLastTimestamp();
    }

    @Override
    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }
}
