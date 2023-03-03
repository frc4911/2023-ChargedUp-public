package com.cyberknights4911.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class ArmIOReal implements ArmIO {
    private static final double ENCODER_DEGREES_PER_ROTATION = 360.0;
    private static final double ENCODER_DEGREES_SHOULDER_OFFSET = -210.0;
    private static final double ENCODER_DEGREES_WRIST_OFFSET = 180.0;

    private final TalonFX shoulderMotor1;
    private final TalonFX shoulderMotor2;
    private final TalonFX shoulderMotor3;
    private final TalonFX wristMotor;


    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder wristEncoder;

    public ArmIOReal() {
        // 1 is closest to robot front (battery side) and the numbering inceases rearward
        shoulderMotor1 =
            TalonFXFactory.createTalon(Ports.Arm.SHOULDER_MOTOR_1, Constants.CANIVORE_NAME);
        shoulderMotor2 =
            TalonFXFactory.createTalon(Ports.Arm.SHOULDER_MOTOR_2, Constants.CANIVORE_NAME);
        shoulderMotor3 =
            TalonFXFactory.createTalon(Ports.Arm.SHOULDER_MOTOR_3, Constants.CANIVORE_NAME);
        wristMotor = TalonFXFactory.createTalon(Ports.Arm.WRIST_MOTOR, Constants.CANIVORE_NAME);

        shoulderEncoder = new DutyCycleEncoder(Ports.Arm.SHOULDER_ENCODER);
        wristEncoder = new DutyCycleEncoder(Ports.Arm.WRIST_ENCODER);

        configMotors();
        configureEncoders();
    }

    private void configMotors() {
        //SHOULDER CONFIGURATION
        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        shoulderConfiguration.supplyCurrLimit.currentLimit = 30.0;
        shoulderConfiguration.statorCurrLimit.currentLimit = 30.0;
        shoulderConfiguration.supplyCurrLimit.enable = true;
        shoulderConfiguration.statorCurrLimit.enable = true;
        
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
        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        wristConfiguration.supplyCurrLimit.currentLimit = 25.0;
        wristConfiguration.statorCurrLimit.currentLimit = 25.0; 

        wristConfiguration.statorCurrLimit.enable = true;
        wristConfiguration.statorCurrLimit.enable = true;

        wristMotor.configAllSettings(wristConfiguration);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    private void configureEncoders() {
        shoulderEncoder.setDistancePerRotation(ENCODER_DEGREES_PER_ROTATION);
        wristEncoder.setDistancePerRotation(ENCODER_DEGREES_PER_ROTATION);
    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO: Calculate and report rotational velocity for encoders
        // TODO: is it useful to log falcon encoder values too?
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
        inputs.wristAppliedVolts = Units.rotationsToRadians(
            wristMotor.getSelectedSensorPosition() / ArmSubsystem.TICKS_PER_REVOLUTION / ArmSubsystem.WRIST_GEAR_RATIO);
        inputs.wristAppliedVolts = shoulderMotor1.getMotorOutputVoltage();
        inputs.wristCurrentAmps = wristMotor.getSupplyCurrent();
        inputs.wristTempCelcius = wristMotor.getTemperature();
    }

    @Override
    public double getWristEncoderDegrees() {
        return (ENCODER_DEGREES_PER_ROTATION - wristEncoder.getDistance() + ENCODER_DEGREES_WRIST_OFFSET) % ENCODER_DEGREES_PER_ROTATION;
    }

    @Override
    public double getShoulderEncoderDegrees() {
        return (ENCODER_DEGREES_PER_ROTATION - shoulderEncoder.getDistance() + ENCODER_DEGREES_SHOULDER_OFFSET) % ENCODER_DEGREES_PER_ROTATION;
    }
    
    @Override
    public boolean isShoulderEncoderConnected() {
        return shoulderEncoder.isConnected();
    }

    @Override
    public boolean isWristEncoderConnected() {
        return wristEncoder.isConnected();
    }

    @Override
    public void setWristBrakeMode() {
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setShoulderBrakeMode() {
        System.out.println("setShoulderBrakeMode");
        shoulderMotor1.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setShoulderOutput(double output) {
        shoulderMotor1.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void setWristOutput(double output) {
        wristMotor.set(ControlMode.PercentOutput, output);
    }

    //This will set the integrated sensors to be accurate with where the arm actually is
    //Remove error introduced by chain
    @Override
    public void adjustError() {
        wristMotor.setSelectedSensorPosition(convertDegreesToTicksWrist(getWristEncoderDegrees()));
        shoulderMotor1.setSelectedSensorPosition(convertDegreesToTicksShoulder(getShoulderEncoderDegrees()));
    }

    // Start of Falcon Sensor Methods
    private double convertDegreesToTicksShoulder(double degrees) {
        return degrees * ArmSubsystem.TICKS_PER_REVOLUTION * ArmSubsystem.SHOULDER_GEAR_RATIO / ArmSubsystem.DEGREES_PER_REVOLUTION;
    }

    private double convertDegreesToTicksWrist(double degrees) {
        return degrees * ArmSubsystem.TICKS_PER_REVOLUTION * ArmSubsystem.WRIST_GEAR_RATIO / ArmSubsystem.DEGREES_PER_REVOLUTION;
    }
    
}
