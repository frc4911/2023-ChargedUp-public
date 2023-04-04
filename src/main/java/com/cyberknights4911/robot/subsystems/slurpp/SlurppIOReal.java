package com.cyberknights4911.robot.subsystems.slurpp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class SlurppIOReal implements SlurppIO {
    private static final double GEAR_RATIO = 1.5;
    private static final double TICKS_PER_REV = 2048;
    
    private final TalonFX motor;

    private double lastStoppedPosition = 0;

    public SlurppIOReal() {
        motor = TalonFXFactory.createTalon(Ports.Slurpp.MOTOR, Constants.CANIVORE_NAME);
        motor.setInverted(false);
        motor.configStatorCurrentLimit(Constants.SLURPP_STATOR_LIMIT, Constants.LONG_CAN_TIMEOUTS_MS);
        motor.configSupplyCurrentLimit(Constants.SLURPP_SUPPLY_LIMIT, Constants.LONG_CAN_TIMEOUTS_MS);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0 , Constants.LONG_CAN_TIMEOUTS_MS);
        
        lastStoppedPosition = motor.getSelectedSensorPosition();
    }

    @Override
    public void updateInputs(SlurppIOInputs inputs) {
        inputs.positionDeg = Units.rotationsToDegrees(
            motor.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.velocityRpm = 
            motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO;
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
        inputs.tempCelcius = motor.getTemperature();
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
        motor.setNeutralMode(NeutralMode.Brake);
        lastStoppedPosition = motor.getSelectedSensorPosition();
    }

    @Override
    public void holdCurrentPosition() {
        motor.set(ControlMode.Position, lastStoppedPosition);
    }
}
