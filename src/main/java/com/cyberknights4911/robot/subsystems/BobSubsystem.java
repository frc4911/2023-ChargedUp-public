package com.cyberknights4911.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import libraries.cyberlib.drivers.TalonFXFactory;

/**
 * Subsystem for controlling the balancer (Bob).
 */
public final class BobSubsystem implements Subsystem {
    
    private Solenoid mBobSolenoid;
    private TalonFX mBobMotor;
    private double ticksAtExtension = 10000; //Random value

    public BobSubsystem() {
        mBobSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Ports.BOB_SOLENOID_PORT);
        mBobMotor = TalonFXFactory.createTalon(Ports.BOB_MOTOR, Constants.CANIVORE_NAME);
        configMotors();
    }

    public void configMotors() {

        //SHOULDER CONFIGURATION
        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        shoulderConfiguration.supplyCurrLimit.currentLimit = 20.0;
        shoulderConfiguration.supplyCurrLimit.enable = true;
        shoulderConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        shoulderConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        shoulderConfiguration.slot0.kI = 0.0;
        shoulderConfiguration.slot0.kD = 0.0;

        mBobMotor.configAllSettings(shoulderConfiguration);
        mBobMotor.setInverted(true);
    }

    public void setLockout (boolean extend) {
        mBobSolenoid.set(extend);
    }

    public void toggleBob(boolean extend) {
        double desiredPosition = 0;
        if (extend) {
            desiredPosition = ticksAtExtension;
        }
        mBobMotor.set(ControlMode.Position, desiredPosition);
    }


}