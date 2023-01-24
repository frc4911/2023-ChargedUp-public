package com.cyberknights4911.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;

public class HoodSubsystem implements Subsystem{
    
    private TalonFX mHoodMotor;

    public enum HoodPositions {
        STOWED(0),
        H1(1000),
        H2(2000);

        double position;

        private HoodPositions(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    private HoodPositions desiredHoodPosition = HoodPositions.STOWED;

    public HoodSubsystem() {

        //1 is closest to robot center and the numbering moves out clockwise
        mHoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR, Constants.CANIVORE_NAME);
        configMotors();
    }

    public void configMotors() {

        //SHOULDER CONFIGURATION
        TalonFXConfiguration ShoulderConfiguration = new TalonFXConfiguration();
        ShoulderConfiguration.supplyCurrLimit.currentLimit = 20.0;
        ShoulderConfiguration.supplyCurrLimit.enable = true;
        ShoulderConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        ShoulderConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        ShoulderConfiguration.slot0.kI = 0.0;
        ShoulderConfiguration.slot0.kD = 0.0;

        
        mHoodMotor.configAllSettings(ShoulderConfiguration);
    }

    public void setDesiredHoodPosition(HoodPositions desiredPosition) {
        desiredHoodPosition = desiredPosition;
        moveHood();
    }

    public void moveHood() {
        mHoodMotor.set(ControlMode.Position, desiredHoodPosition.get());
    }
}

