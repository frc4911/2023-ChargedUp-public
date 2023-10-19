package com.cyberknights4911.robot.model.wham.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.cyberknights4911.robot.model.wham.WhamConstants;
import com.cyberknights4911.robot.model.wham.WhamPorts;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class ArmIOReal implements ArmIO {
    private final WPI_TalonFX shoulderMotor1;
    private final WPI_TalonFX shoulderMotor2;
    private final WPI_TalonFX shoulderMotor3;
    private final WPI_TalonFX wristMotor;
    private final WPI_CANCoder shoulderEncoder;
    private final WPI_CANCoder wristEncoder;
    private final CtreError ctreError;

    public ArmIOReal(TalonFXFactory talonFXFactory, CANCoderFactory canCoderFactory, CtreError ctreError) {
        this.ctreError = ctreError;

        // 1 is closest to robot front (battery side) and the numbering inceases rearward
        shoulderMotor1 = talonFXFactory.createTalon(WhamPorts.Arm.SHOULDER_MOTOR_1);
        shoulderMotor2 = talonFXFactory.createTalon(WhamPorts.Arm.SHOULDER_MOTOR_2);
        shoulderMotor3 = talonFXFactory.createTalon(WhamPorts.Arm.SHOULDER_MOTOR_3);
        wristMotor = talonFXFactory.createTalon(WhamPorts.Arm.WRIST_MOTOR);

        shoulderEncoder = canCoderFactory.create(WhamPorts.Arm.SHOULDER_CANCODER);
        wristEncoder = canCoderFactory.create(WhamPorts.Arm.WRIST_CANCODER);

        configMotors();
        configureEncoders();
    }

    private void configMotors() {
        //SHOULDER CONFIGURATION
        ctreError.checkError(shoulderMotor1.configRemoteFeedbackFilter(
            shoulderEncoder, WhamConstants.REMOTE_SENSOR_ZERO, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.configSelectedFeedbackSensor(
            FeedbackDevice.RemoteSensor0, WhamConstants.PRIMARY_PID, ctreError.canTimeoutMs()));
        shoulderMotor1.setSensorPhase(true);

        TalonFXConfiguration shoulderConfiguration1 = new TalonFXConfiguration();
        TalonFXConfiguration shoulderConfigurationFollowers = new TalonFXConfiguration();

        shoulderConfiguration1.supplyCurrLimit = WhamConstants.Arm.SHOULDER_SUPPLY_LIMIT;
        shoulderConfiguration1.statorCurrLimit = WhamConstants.Arm.SHOULDER_STATOR_LIMIT;
        shoulderConfiguration1.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        shoulderConfiguration1.remoteFilter0 = WhamConstants.Arm.SHOULDER_FILTER_CONFIG;
        shoulderConfiguration1.slot0 = WhamConstants.Arm.SHOULDER_SLOT_CONFIG;
        shoulderConfiguration1.neutralDeadband = WhamConstants.Arm.SHOULDER_NEUTRAL_DEADBAND.getValue();

        shoulderConfigurationFollowers.supplyCurrLimit = WhamConstants.Arm.SHOULDER_SUPPLY_LIMIT;
        shoulderConfigurationFollowers.statorCurrLimit = WhamConstants.Arm.SHOULDER_STATOR_LIMIT;

        ctreError.checkError(shoulderMotor1.configAllSettings(shoulderConfiguration1, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, WhamConstants.PRIMARY_PID_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, WhamConstants.MOTION_MAGIC_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, WhamConstants.PRIMARY_PID0_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.configMotionCruiseVelocity(WhamConstants.Arm.SHOULDER_VELOCITY_MOTION_MAGIC.getValue(), ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor1.configMotionAcceleration(WhamConstants.Arm.SHOULDER_ACCELERATION_MOTION_MAGIC.getValue(), ctreError.canTimeoutMs()));

        ctreError.checkError(shoulderMotor2.configAllSettings(shoulderConfigurationFollowers, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor3.configAllSettings(shoulderConfigurationFollowers, ctreError.canTimeoutMs()));
        shoulderMotor2.follow(shoulderMotor1);
        shoulderMotor3.follow(shoulderMotor1);
        shoulderMotor1.setInverted(true);
        shoulderMotor2.setInverted(InvertType.FollowMaster);
        shoulderMotor3.setInverted(InvertType.FollowMaster);
        ctreError.checkError(shoulderMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, ctreError.canTimeoutMs()));
        ctreError.checkError(shoulderMotor3.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, ctreError.canTimeoutMs()));

        shoulderMotor1.setNeutralMode(NeutralMode.Coast);
        shoulderMotor2.setNeutralMode(NeutralMode.Coast);
        shoulderMotor3.setNeutralMode(NeutralMode.Coast);

        //WRIST CONFIGURATION
        ctreError.checkError(wristMotor.configRemoteFeedbackFilter(
            wristEncoder, WhamConstants.REMOTE_SENSOR_ZERO, ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.configSelectedFeedbackSensor(
            FeedbackDevice.RemoteSensor0, WhamConstants.PRIMARY_PID, ctreError.canTimeoutMs()));
        wristMotor.setSensorPhase(true);

        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        wristConfiguration.supplyCurrLimit = WhamConstants.Arm.WRIST_SUPPLY_LIMIT;
        wristConfiguration.statorCurrLimit = WhamConstants.Arm.WRIST_STATOR_LIMIT;
        wristConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        wristConfiguration.remoteFilter0 = WhamConstants.Arm.WRIST_FILTER_CONFIG;
        wristConfiguration.slot0 = WhamConstants.Arm.WRIST_SLOT_CONFIG;
        wristConfiguration.neutralDeadband = WhamConstants.Arm.WRIST_NEUTRAL_DEADBAND.getValue();

        ctreError.checkError(wristMotor.configAllSettings(wristConfiguration, ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, WhamConstants.PRIMARY_PID_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, WhamConstants.MOTION_MAGIC_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, WhamConstants.PRIMARY_PID0_PERIOD, ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.configMotionCruiseVelocity(WhamConstants.Arm.WRIST_VELOCITY_MOTION_MAGIC.getValue(), ctreError.canTimeoutMs()));
        ctreError.checkError(wristMotor.configMotionAcceleration(WhamConstants.Arm.WRIST_ACCELERATION_MOTION_MAGIC.getValue(), ctreError.canTimeoutMs()));
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    private void configureEncoders() {
        CANCoderConfiguration shoulderConfig = new CANCoderConfiguration();
        shoulderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        shoulderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        shoulderConfig.magnetOffsetDegrees = WhamConstants.Arm.SHOULDER_CANCODER_OFFSET.getValue();
        shoulderConfig.sensorDirection = false;

        ctreError.checkError(shoulderEncoder.configAllSettings(shoulderConfig, ctreError.canTimeoutMs()));

        CANCoderConfiguration wristConfig = new CANCoderConfiguration();
        wristConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        wristConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        wristConfig.magnetOffsetDegrees = WhamConstants.Arm.WRIST_CANCODER_OFFSET.getValue();
        wristConfig.sensorDirection = false;

        ctreError.checkError(wristEncoder.configAllSettings(wristConfig, ctreError.canTimeoutMs()));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO: Calculate and report rotational velocity for encoders
        inputs.shoulderPositionDeg = getShoulderEncoderDegrees();
        inputs.shoulderVelocityUnitsPerHundredMs = shoulderMotor1.getSelectedSensorVelocity();
        inputs.shoulderSelectedSensorPosition = shoulderMotor1.getSelectedSensorPosition();
        inputs.shoulderRemoteEncoderPosition = ArmSubsystem.convertDegreesToCtreTicks(shoulderEncoder.getAbsolutePosition());
        inputs.shoulderAppliedVolts = new double[] {
            shoulderMotor1.getMotorOutputVoltage(),
            shoulderMotor2.getMotorOutputVoltage(),
            shoulderMotor3.getMotorOutputVoltage()
        };
        inputs.shoulderCurrentAmps = new double[] {
            shoulderMotor1.getSupplyCurrent(),
            shoulderMotor2.getSupplyCurrent(),
            shoulderMotor3.getSupplyCurrent()
        };
        inputs.shoulderTempCelcius = new double[] {
            shoulderMotor1.getTemperature(),
            shoulderMotor2.getTemperature(),
            shoulderMotor3.getTemperature()
        };

        inputs.wristPositionDeg = getWristEncoderDegrees();
        inputs.wristVelocityDegPerSec = wristMotor.getSelectedSensorVelocity();
        inputs.wristSelectedSensorPosition = wristMotor.getSelectedSensorPosition();
        inputs.wristAppliedVolts = wristMotor.getMotorOutputVoltage();
        inputs.wristCurrentAmps = wristMotor.getSupplyCurrent();
        inputs.wristTempCelcius = wristMotor.getTemperature();
    }

    @Override
    public double getShoulderEncoderDegrees() {
        return shoulderEncoder.getAbsolutePosition();
    }

    @Override
    public double getWristEncoderDegrees() {
        return wristEncoder.getAbsolutePosition();
    }

    @Override
    public void setShoulderPosition(double position) {
        // TODO: reject positions beyond soft stops
        if (WhamConstants.Arm.SHOULD_USE_GRAVITY_FEED_FORWARD.getValue()) {
            double degrees = getShoulderEncoderDegrees() - 90;
            double cosineScalar = java.lang.Math.cos(Math.toRadians(degrees));
            shoulderMotor1.set(
                ControlMode.MotionMagic,
                position,
                DemandType.ArbitraryFeedForward,
                cosineScalar * WhamConstants.Arm.SHOULDER_G.getValue());
        } else {
            shoulderMotor1.set(ControlMode.MotionMagic, position);
        }
    }

    @Override
    public void setWristPosition(double position) {
        // TODO: reject positions beyond soft stops
        wristMotor.set(ControlMode.MotionMagic, position);
    }

    @Override
    public double getShoulderTrajectoryPosition() {
        return shoulderMotor1.getActiveTrajectoryPosition();
    }

    @Override
    public double getWristTrajectoryPosition() {
        return wristMotor.getActiveTrajectoryPosition();
    }

}
