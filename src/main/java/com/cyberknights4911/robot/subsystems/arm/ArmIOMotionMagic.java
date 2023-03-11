package com.cyberknights4911.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
            shoulderEncoder, 0, Constants.LONG_CAN_TIMEOUTS_MS);

        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        shoulderConfiguration.supplyCurrLimit.currentLimit = 30.0;
        shoulderConfiguration.statorCurrLimit.currentLimit = 30.0;
        shoulderConfiguration.supplyCurrLimit.enable = true;
        shoulderConfiguration.statorCurrLimit.enable = true;
        shoulderConfiguration.primaryPID.selectedFeedbackSensor =
            TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
        // TODO: tune these values
        shoulderConfiguration.slot0.kP = 0.0;
        shoulderConfiguration.slot0.kI = 0.0;
        shoulderConfiguration.slot0.kD = 0.0;
        
        shoulderMotor1.configAllSettings(shoulderConfiguration);
        shoulderMotor2.follow(shoulderMotor1);
        shoulderMotor3.follow(shoulderMotor1);
        shoulderMotor1.setInverted(true);
        shoulderMotor2.setInverted(InvertType.FollowMaster);
        shoulderMotor3.setInverted(InvertType.FollowMaster);
        shoulderMotor1.setNeutralMode(NeutralMode.Brake);
        shoulderMotor2.setNeutralMode(NeutralMode.Brake);
        shoulderMotor3.setNeutralMode(NeutralMode.Brake);

        //WRIST CONFIGURATION
        wristMotor.configRemoteFeedbackFilter(
            wristEncoder, 0, Constants.LONG_CAN_TIMEOUTS_MS);

        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        wristConfiguration.supplyCurrLimit.currentLimit = 30.0;
        wristConfiguration.statorCurrLimit.currentLimit = 30.0; 
        wristConfiguration.statorCurrLimit.enable = true;
        wristConfiguration.statorCurrLimit.enable = true;
        wristConfiguration.primaryPID.selectedFeedbackSensor =
            TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
        // TODO: tune these values
        wristConfiguration.slot0.kP = 0.0;
        wristConfiguration.slot0.kI = 0.0;
        wristConfiguration.slot0.kD = 0.0;

        wristMotor.configAllSettings(wristConfiguration);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    private void configureEncoders() {
        CANCoderConfiguration shoulderConfig = new CANCoderConfiguration();
        shoulderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        shoulderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // TODO: determine magnet offset after installation
        shoulderConfig.magnetOffsetDegrees = 0;
        shoulderConfig.sensorDirection = false;

        shoulderEncoder.configAllSettings(shoulderConfig, Constants.LONG_CAN_TIMEOUTS_MS);
        
        CANCoderConfiguration wristConfig = new CANCoderConfiguration();
        wristConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        wristConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // TODO: determine magnet offset after installation
        wristConfig.magnetOffsetDegrees = 0;
        wristConfig.sensorDirection = false;

        wristEncoder.configAllSettings(wristConfig, Constants.LONG_CAN_TIMEOUTS_MS);
    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO: Calculate and report rotational velocity for encoders
        inputs.shoulderPositionDeg = getShoulderEncoderDegrees();
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
    
}
