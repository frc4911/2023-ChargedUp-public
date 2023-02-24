package com.cyberknights4911.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class ArmIOReal implements ArmIO {

    private final TalonFX shoulderMotor1;
    private final TalonFX shoulderMotor2;
    private final TalonFX shoulderMotor3;
    private final TalonFX wristMotor;


    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder wristEncoder;

    public ArmIOReal() {
        // 1 is closest to robot center and the numbering moves out clockwise
        shoulderMotor1 =
            TalonFXFactory.createTalon(Ports.SHOULDER_MOTOR_1, Constants.CANIVORE_NAME);
        shoulderMotor2 =
            TalonFXFactory.createTalon(Ports.SHOULDER_MOTOR_2, Constants.CANIVORE_NAME);
        shoulderMotor3 =
            TalonFXFactory.createTalon(Ports.SHOULDER_MOTOR_3, Constants.CANIVORE_NAME);
        wristMotor = TalonFXFactory.createTalon(Ports.WRIST_MOTOR, Constants.CANIVORE_NAME);

        shoulderEncoder = new DutyCycleEncoder(Ports.ARM_SHOULDER_ENCODER);
        shoulderEncoder.setPositionOffset(0.43);

        wristEncoder = new DutyCycleEncoder(Ports.ARM_WRIST_ENCODER);
        wristEncoder.setPositionOffset(0.18);

        configMotors();
    }
    
    private void configMotors() {
        //SHOULDER CONFIGURATION
        //May need to use a wpilib pid controller instead if we are not going to use an encoder
        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        //shoulderConfiguration.supplyCurrLimit.currentLimit = 20.0;
        shoulderConfiguration.supplyCurrLimit.currentLimit = 10.0;
        shoulderConfiguration.statorCurrLimit.currentLimit = 3.0;
        shoulderConfiguration.supplyCurrLimit.enable = true;
        shoulderConfiguration.statorCurrLimit.enable = true;
        shoulderConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        shoulderConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        shoulderConfiguration.slot0.kI = 0.0;
        shoulderConfiguration.slot0.kD = 0.0;
        shoulderConfiguration.slot0.kF = 0.0; // No idea if this is neccessary
        
        shoulderMotor3.configAllSettings(shoulderConfiguration);
        shoulderMotor1.follow(shoulderMotor3);
        shoulderMotor2.follow(shoulderMotor3);
        shoulderMotor3.setInverted(true);
        shoulderMotor2.setInverted(InvertType.FollowMaster);
        shoulderMotor1.setInverted(InvertType.FollowMaster);

        //TODO: May want to use setStatusFramePeriod to be lower and make motors followers if having CAN utilization issues
        shoulderMotor3.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shoulderMotor3.setSelectedSensorPosition(convertDegreesToTicksShoulder(getShoulderDegrees()));

        //WRIST CONFIGURATION
        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        //wristConfiguration.supplyCurrLimit.currentLimit = 20.0;
        wristConfiguration.statorCurrLimit.currentLimit = 10.0; //TODO:30 amps

        wristConfiguration.statorCurrLimit.enable = true;
        wristConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        wristConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        wristConfiguration.slot0.kI = 0.0;
        wristConfiguration.slot0.kD = 0.0;
        wristConfiguration.slot0.kF = 0.0; // No idea if this is neccessary

        wristMotor.configAllSettings(wristConfiguration);
        wristMotor.setSelectedSensorPosition(convertDegreesToTicksWrist(getWristDegrees()));
        wristMotor.setInverted(false);


    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderPositionRad = Units.rotationsToRadians(
            shoulderMotor1.getSelectedSensorPosition() / ArmSubsystem.TICKS_PER_REVOLUTION / ArmSubsystem.SHOULDER_GEAR_RATIO);
        inputs.shoulderVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            shoulderMotor1.getSelectedSensorVelocity() * 10 / ArmSubsystem.TICKS_PER_REVOLUTION / ArmSubsystem.SHOULDER_GEAR_RATIO);
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

        inputs.wristAppliedVolts = Units.rotationsToRadians(
            wristMotor.getSelectedSensorPosition() / ArmSubsystem.TICKS_PER_REVOLUTION / ArmSubsystem.WRIST_GEAR_RATIO);
        inputs.wristVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            shoulderMotor1.getSelectedSensorVelocity() * 10 / ArmSubsystem.TICKS_PER_REVOLUTION / ArmSubsystem.WRIST_GEAR_RATIO);
        inputs.wristAppliedVolts = shoulderMotor1.getMotorOutputVoltage();
        inputs.wristCurrentAmps = wristMotor.getSupplyCurrent();
        inputs.wristTempCelcius = wristMotor.getTemperature();
    }

    @Override
    public double getWristPositionEncoder() {
        return Math.abs(wristEncoder.get());
    }

    @Override
    public double getShoulderPositionEncoder() {
        return Math.abs(shoulderEncoder.get());
    }
    
    @Override
    public void setWristBrakeMode() {
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setShoulderBrakeMode() {
        shoulderMotor3.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setShoulderPosition(double position) {
        shoulderMotor3.set(ControlMode.Position, position);
    }

    @Override
    public void setWristPosition(double position) {
        wristMotor.set(ControlMode.Position, position);
    }

    //This will set the integrated sensors to be accurate with where the arm actually is
    //Remove error introduced by chain
    @Override
    public void adjustError() {
        wristMotor.setSelectedSensorPosition(convertDegreesToTicksWrist(getWristDegrees()));
        shoulderMotor3.setSelectedSensorPosition(convertDegreesToTicksShoulder(getShoulderDegrees()));
    }

    //The following are all for the Absolute Encoder not the Integrated Falcon Sensor
    //TODO:Use these values to setSelectedSensorPosition() of leader falcon when it gets inaccurate
    public double getShoulderDegrees() {
        return dutyCycleToDegrees(getShoulderPositionEncoder());
      }

      public double getWristDegrees() {
        return dutyCycleToDegrees(getWristPositionEncoder());
      }

      public double dutyCycleToDegrees(double dutyCyclePos) {
        return dutyCyclePos * 360;
      }

      //Start of Falcon Sensor Methods
    private double convertDegreesToTicksShoulder(double degrees) {
        return degrees * ArmSubsystem.TICKS_PER_REVOLUTION * ArmSubsystem.SHOULDER_GEAR_RATIO / ArmSubsystem.DEGREES_PER_REVOLUTION;
    }

    private double convertDegreesToTicksWrist(double degrees) {
        return degrees * ArmSubsystem.TICKS_PER_REVOLUTION * ArmSubsystem.WRIST_GEAR_RATIO / ArmSubsystem.DEGREES_PER_REVOLUTION;
    }
    
}
