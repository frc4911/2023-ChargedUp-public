package com.cyberknights4911.robot.subsystems.slurpp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class SlurppIOReal implements SlurppIO {
    private static final double GEAR_RATIO = 1.5;
    private static final double TICKS_PER_REV = 2048;
    
    private final TalonFX motor;

    public SlurppIOReal() {
        motor = TalonFXFactory.createTalon(Ports.SLURPP_MOTOR, Constants.CANIVORE_NAME);
        motor.setInverted(false);
    }

    @Override
    public void updateInputs(SplurppIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(
            motor.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
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
    }
}
