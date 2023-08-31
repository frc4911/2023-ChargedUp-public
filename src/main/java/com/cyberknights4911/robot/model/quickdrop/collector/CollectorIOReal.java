package com.cyberknights4911.robot.model.quickdrop.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.cyberknights4911.robot.model.quickdrop.QuickDropPorts;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class CollectorIOReal implements CollectorIO {
    private static final double TICKS_PER_REV = 2048;

    private final WPI_TalonFX motor;
    private final Solenoid solenoid;
    private final CtreError ctreError;

    public CollectorIOReal(TalonFXFactory talonFXFactory, CtreError ctreError) {
        this.ctreError = ctreError;
        motor = talonFXFactory.createTalon(QuickDropPorts.Collector.MOTOR);
        configMotors();

        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, QuickDropPorts.Collector.SOLENOID);
    }

    private void configMotors() {
        ctreError.checkError(motor.configFactoryDefault(ctreError.canTimeoutMs()));
        motor.setInverted(false);
    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {
        inputs.velocityRpm = motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV;
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
        inputs.tempCelcius = motor.getTemperature();
        inputs.isExtended = solenoid.get();
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
        motor.setNeutralMode(NeutralMode.Brake);
    }
    
    @Override
    public void extend() {
        solenoid.set(true);
    }
    
    @Override
    public void retract() {
        solenoid.set(false);
    }
}
