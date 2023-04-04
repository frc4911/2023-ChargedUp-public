package com.cyberknights4911.robot.subsystems.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class HoodIOReal implements HoodIO {
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 2048;

    private final TalonFX motor;
    
    public HoodIOReal() {
        motor = TalonFXFactory.createTalon(Ports.Robot2022Hood.MOTOR, Constants.CANIVORE_NAME);
        configMotors();
    }

    public void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.currentLimit = 10.0;
        config.supplyCurrLimit.enable = true;
        config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        config.slot0.kP = 0.25; //Default PID values no rhyme or reason
        config.slot0.kI = 0.0;
        config.slot0.kD = 0.0;
        
        motor.configAllSettings(config);
        motor.setInverted(true);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(
            motor.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
        inputs.tempCelcius = motor.getTemperature();
    }

    @Override
    public void setPosition(double position) {
        motor.set(ControlMode.Position, position);
    }
}
