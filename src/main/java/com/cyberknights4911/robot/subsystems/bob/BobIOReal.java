package com.cyberknights4911.robot.subsystems.bob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class BobIOReal implements BobIO {
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 2048;

    private final Solenoid solenoid;
    private final TalonFX motor;
    
    public BobIOReal() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.Bob.SOLENOID);
        motor = TalonFXFactory.createTalon(Ports.Bob.MOTOR, Constants.CANIVORE_NAME);

        solenoid.set(false);
        configureMotor();
    }
    
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.currentLimit = 20.0;
        config.supplyCurrLimit.enable = true;
        config.primaryPID.selectedFeedbackSensor =
            TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        config.slot0.kP = 0.25; //Default PID values no rhyme or reason
        config.slot0.kI = 0.0;
        config.slot0.kD = 0.0;

        motor.configAllSettings(config);
        motor.setInverted(true);
    }

    @Override
    public void updateInputs(BobIOInputs inputs) {
        inputs.extended = solenoid.get();
        
        inputs.positionDeg = Units.rotationsToDegrees(
            motor.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.velocityRpm =
            motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO;
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
        inputs.tempCelcius = motor.getTemperature();
    }

    @Override
    public void setExtended(boolean extended) {
        solenoid.set(extended);
    }

    @Override
    public void setPosition(double position) {
        motor.set(ControlMode.Position, position);
    }
}
