package com.cyberknights4911.robot.model.quickdrop.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.cyberknights4911.robot.model.quickdrop.QuickDropPorts;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class CollectorIOReal implements CollectorIO {
    
    private final WPI_TalonFX motor;
    private final Solenoid solenoid;

    public CollectorIOReal(TalonFXFactory talonFXFactory) {
        motor = talonFXFactory.createTalon(QuickDropPorts.Collector.MOTOR);
        motor.setInverted(false);
        // motor.configStatorCurrentLimit(Constants.SLURPP_STATOR_LIMIT, Constants.LONG_CAN_TIMEOUTS_MS);
        // motor.configSupplyCurrentLimit(Constants.SLURPP_SUPPLY_LIMIT, Constants.LONG_CAN_TIMEOUTS_MS);
        // motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0 , Constants.LONG_CAN_TIMEOUTS_MS);
        
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, QuickDropPorts.Collector.SOLENOID);
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
