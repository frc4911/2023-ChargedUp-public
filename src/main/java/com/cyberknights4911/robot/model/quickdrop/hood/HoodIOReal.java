package com.cyberknights4911.robot.model.quickdrop.hood;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.model.quickdrop.QuickDropPorts;
import edu.wpi.first.math.util.Units;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class HoodIOReal implements HoodIO {
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 2048;

    private final TalonFX motor;

    public static Supplier<HoodIO> getSupplier(TalonFXFactory talonFXFactory) {
        return () -> {
            return new HoodIOReal(talonFXFactory);
        };
    }

    private HoodIOReal(TalonFXFactory talonFXFactory) {
        motor = talonFXFactory.createTalon(QuickDropPorts.Hood.MOTOR);
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
