package com.cyberknights4911.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import libraries.cyberlib.drivers.TalonFXFactory;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

public class HoodSubsystem extends SubsystemBase{
    
    private TalonFX mHoodMotor;

    public enum HoodPositions {
        STOWED(0),
        H1(10000),
        H2(20000);

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
        mHoodMotor = TalonFXFactory.createTalon(Ports.ROBOT_2022_HOOD_MOTOR, Constants.CANIVORE_NAME);
        configMotors();
    }

    public void configMotors() {

        //SHOULDER CONFIGURATION
        TalonFXConfiguration ShoulderConfiguration = new TalonFXConfiguration();
        ShoulderConfiguration.supplyCurrLimit.currentLimit = 10.0;
        ShoulderConfiguration.supplyCurrLimit.enable = true;
        ShoulderConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        ShoulderConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        ShoulderConfiguration.slot0.kI = 0.0;
        ShoulderConfiguration.slot0.kD = 0.0;

        
        mHoodMotor.configAllSettings(ShoulderConfiguration);
        mHoodMotor.setInverted(true);
    }

    public void setDesiredHoodPosition(HoodPositions desiredPosition) {
        desiredHoodPosition = desiredPosition;
        moveHood();
    }

    public void moveHood() {
        mHoodMotor.set(ControlMode.Position, desiredHoodPosition.get());
    }

    @Override
    public void periodic() {
        
    }

}

