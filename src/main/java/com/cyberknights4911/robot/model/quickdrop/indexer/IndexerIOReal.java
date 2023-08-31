package com.cyberknights4911.robot.model.quickdrop.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.cyberknights4911.robot.model.quickdrop.QuickDropPorts;

import edu.wpi.first.wpilibj.AnalogInput;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.TalonFXFactory;

public class IndexerIOReal implements IndexerIO {

    private final WPI_TalonFX motor;
    private final AnalogInput enter;
    private final AnalogInput exit;
    private final CtreError ctreError;

    public IndexerIOReal(TalonFXFactory talonFXFactory, CtreError ctreError) {
        this.ctreError = ctreError;
        motor = talonFXFactory.createTalon(QuickDropPorts.Indexer.MOTOR);
        configMotors();
        enter = new AnalogInput(QuickDropPorts.Indexer.BEAM_BREAK_ENTER);
        exit = new AnalogInput(QuickDropPorts.Indexer.BEAM_BREAK_EXIT);
    }
    
    private void configMotors() {
        ctreError.checkError(motor.configFactoryDefault(ctreError.canTimeoutMs()));
        motor.setInverted(true);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.enterVoltage = enter.getVoltage();
        inputs.exitVoltage = exit.getVoltage();
        inputs.velocityRpm = motor.getSelectedSensorVelocity() * 10 / 2048;
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
    public boolean isEnterBlocked() {
        return enter.getVoltage() < 3.0;
    }

    @Override
    public boolean isExitBlocked() {
        return exit.getVoltage() < 3.0;
    }
}
