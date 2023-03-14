package com.cyberknights4911.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports.Arm;

import libraries.cyberlib.drivers.TalonFXFactory;

public final class ArmIOMotionMagic implements ArmIO {

    private final TalonFX shoulderMotor1;
    private final TalonFX shoulderMotor2;
    private final TalonFX shoulderMotor3;
    private final TalonFX wristMotor;
    
    private final CANCoder shoulderEncoder;
    private final CANCoder wristEncoder;

    public ArmIOMotionMagic() {
        // 1 is closest to robot front (battery side) and the numbering inceases rearward
        shoulderMotor1 =
            TalonFXFactory.createTalon(Arm.SHOULDER_MOTOR_1, Constants.CANIVORE_NAME);
        shoulderMotor2 =
            TalonFXFactory.createTalon(Arm.SHOULDER_MOTOR_2, Constants.CANIVORE_NAME);
        shoulderMotor3 =
            TalonFXFactory.createTalon(Arm.SHOULDER_MOTOR_3, Constants.CANIVORE_NAME);
        wristMotor = TalonFXFactory.createTalon(Arm.WRIST_MOTOR, Constants.CANIVORE_NAME);

        shoulderEncoder = new CANCoder(Arm.SHOULDER_CANCODER, Constants.CANIVORE_NAME);
        wristEncoder = new CANCoder(Arm.WRIST_CANCODER, Constants.CANIVORE_NAME);

        configMotors();
        configureEncoders();
    }

    private void configMotors() {
        //SHOULDER CONFIGURATION
        shoulderMotor1.configRemoteFeedbackFilter(
            shoulderEncoder, Constants.REMOTE_SENSOR_ZERO, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.configSelectedFeedbackSensor(
            FeedbackDevice.RemoteSensor0, Constants.PRIMARY_PID, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.setSensorPhase(true);

        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        shoulderConfiguration.supplyCurrLimit = Constants.Arm.SHOULDER_SUPPLY_LIMIT;
        shoulderConfiguration.statorCurrLimit = Constants.Arm.SHOULDER_STATOR_LIMIT;
        shoulderConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        shoulderConfiguration.remoteFilter0 = Constants.Arm.SHOULDER_FILTER_CONFIG;
        shoulderConfiguration.slot0 = Constants.Arm.SHOULDER_SLOT_CONFIG;
        shoulderConfiguration.neutralDeadband = Constants.Arm.SHOULDER_NEUTRAL_DEADBAND;
        
        shoulderMotor1.configAllSettings(shoulderConfiguration);
        shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.PRIMARY_PID_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, Constants.MOTION_MAGIC_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.PRIMARY_PID0_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.configMotionCruiseVelocity(Constants.Arm.SHOULDER_VELOCITY_MOTION_MAGIC, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor1.configMotionAcceleration(Constants.Arm.SHOULDER_ACCELERATION_MOTION_MAGIC, Constants.LONG_CAN_TIMEOUTS_MS);
        shoulderMotor2.follow(shoulderMotor1);
        shoulderMotor3.follow(shoulderMotor1);
        shoulderMotor1.setInverted(true);
        shoulderMotor2.setInverted(InvertType.FollowMaster);
        shoulderMotor3.setInverted(InvertType.FollowMaster);
        shoulderMotor1.setNeutralMode(NeutralMode.Coast);
        shoulderMotor2.setNeutralMode(NeutralMode.Coast);
        shoulderMotor3.setNeutralMode(NeutralMode.Coast);

        //WRIST CONFIGURATION
        wristMotor.configRemoteFeedbackFilter(
            wristEncoder, Constants.REMOTE_SENSOR_ZERO, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.configSelectedFeedbackSensor(
            FeedbackDevice.RemoteSensor0, Constants.PRIMARY_PID, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.setSensorPhase(true);

        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        wristConfiguration.supplyCurrLimit = Constants.Arm.WRIST_SUPPLY_LIMIT;
        wristConfiguration.statorCurrLimit = Constants.Arm.WRIST_STATOR_LIMIT;
        wristConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        wristConfiguration.remoteFilter0 = Constants.Arm.WRIST_FILTER_CONFIG;
        wristConfiguration.slot0 = Constants.Arm.WRIST_SLOT_CONFIG;
        wristConfiguration.neutralDeadband = Constants.Arm.WRIST_NEUTRAL_DEADBAND;
        
        wristMotor.configAllSettings(wristConfiguration);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.PRIMARY_PID_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, Constants.MOTION_MAGIC_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.PRIMARY_PID0_PERIOD, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.configMotionCruiseVelocity(Constants.Arm.WRIST_VELOCITY_MOTION_MAGIC, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.configMotionAcceleration(Constants.Arm.WRIST_ACCELERATION_MOTION_MAGIC, Constants.LONG_CAN_TIMEOUTS_MS);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    private void configureEncoders() {
        CANCoderConfiguration shoulderConfig = new CANCoderConfiguration();
        shoulderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        shoulderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        shoulderConfig.magnetOffsetDegrees = Constants.Arm.SHOULDER_CANCODER_OFFSET;
        shoulderConfig.sensorDirection = false;

        shoulderEncoder.configAllSettings(shoulderConfig, Constants.LONG_CAN_TIMEOUTS_MS);
        
        CANCoderConfiguration wristConfig = new CANCoderConfiguration();
        wristConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        wristConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        wristConfig.magnetOffsetDegrees = Constants.Arm.WRIST_CANCODER_OFFSET;
        wristConfig.sensorDirection = false;

        wristEncoder.configAllSettings(wristConfig, Constants.LONG_CAN_TIMEOUTS_MS);
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
        shoulderMotor1.set(ControlMode.MotionMagic, position);
        // double feedForward = 0.0;
        // TODO: calculate feed forward according to:
        // https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html?highlight=motion%20magic#gravity-offset-arm
        //shoulderMotor1.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, feedForward);
    }

    @Override
    public void setWristPosition(double position) {
        // TODO: reject positions beyond soft stops
        wristMotor.set(ControlMode.MotionMagic, position);
    }

    // Set output should not be generally used, but are left here for testing
    @Override
    public void setWristOutput(double output) {
        wristMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void setShoulderOutput(double output) {
        shoulderMotor1.set(ControlMode.PercentOutput, output);
    }
}
