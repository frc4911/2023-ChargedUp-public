package com.cyberknights4911.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import libraries.cheesylib.drivers.TalonFXFactory;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem implements Subsystem {

    private TalonFX mShoulderMotor1;
    private TalonFX mShoulderMotor2;
    private TalonFX mShoulderMotor3;
    private TalonFX mShoulderMotor4;
    private TalonFX mWristMotor;

    //In ticks can be changed to be in degrees
    //Makes it run much faster because it does not need to be precise
    private double ARM_ERROR = 100;
    private double WRIST_ERROR = 100;

    public enum ArmPositions {
        STOWED(0),
        CONE_LEVEL_3(240), 
        CUBE_LEVEL_3(240),
        CONE_LEVEL_2(270),
        CUBE_LEVEL_2(270),
        HYBRID_CONE(300),
        HYBRID_CUBE(300),
        COLLECT_PORTAL(240),
        COLLECT_GROUND(300);

        double position;

        private ArmPositions(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    private ArmPositions desiredShoulderPosition = ArmPositions.STOWED;

    public ArmSubsystem() {

        //1 is closest to robot center and the numbering moves out clockwise
        mShoulderMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_1, Constants.CANIVORE_NAME);
        mShoulderMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_2, Constants.CANIVORE_NAME);
        mShoulderMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_3, Constants.CANIVORE_NAME);
        mShoulderMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_4, Constants.CANIVORE_NAME);
        mWristMotor = TalonFXFactory.createDefaultTalon(Ports.WRIST_MOTOR, Constants.CANIVORE_NAME);
        configMotors();
    }

    public void configMotors() {

        //SHOULDER CONFIGURATION
        //May need to use a wpilib pid controller instead if we are not going to use an encoder
        TalonFXConfiguration shoulderConfiguration = new TalonFXConfiguration();
        shoulderConfiguration.supplyCurrLimit.currentLimit = 20.0;
        shoulderConfiguration.supplyCurrLimit.enable = true;
        shoulderConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        shoulderConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        shoulderConfiguration.slot0.kI = 0.0;
        shoulderConfiguration.slot0.kD = 0.0;
        shoulderConfiguration.slot0.kF = 0.0; // No idea if this is neccessary

        
        mShoulderMotor1.configAllSettings(shoulderConfiguration);
        mShoulderMotor2.configAllSettings(shoulderConfiguration);
        mShoulderMotor3.configAllSettings(shoulderConfiguration);
        mShoulderMotor4.configAllSettings(shoulderConfiguration);

        mShoulderMotor3.setInverted(true);
        mShoulderMotor4.setInverted(true);

        //May want to use setStatusFramePeriod to be lower and make motors followers if having CAN utilization issues
        mShoulderMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        //WRIST CONFIGURATION
        TalonFXConfiguration wristConfiguration = new TalonFXConfiguration();
        wristConfiguration.supplyCurrLimit.currentLimit = 20.0;
        wristConfiguration.supplyCurrLimit.enable = true;
        wristConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        wristConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        wristConfiguration.slot0.kI = 0.0;
        wristConfiguration.slot0.kD = 0.0;
        wristConfiguration.slot0.kF = 0.0; // No idea if this is neccessary


        mWristMotor.configAllSettings(shoulderConfiguration);


    }

    public void setShoulderDesiredPosition(ArmPositions desiredPosition) {
        desiredShoulderPosition = desiredPosition;
    }

    public void moveShoulder() {
        double falconTicks = convertDegreesToTicksShoulder(desiredShoulderPosition.get());
        mShoulderMotor1.set(ControlMode.Position, falconTicks);
        mShoulderMotor2.set(ControlMode.Position, falconTicks);
        mShoulderMotor3.set(ControlMode.Position, falconTicks);
        mShoulderMotor4.set(ControlMode.Position, falconTicks);
    }

    public void moveWrist() {
        double falconTicks = desiredWristPosition();
        mWristMotor.set(ControlMode.Position, falconTicks);
    }

    public double desiredWristPosition() {
        return convertDegreesToTicksWrist(desiredShoulderPosition.get()%180);
    }

    public boolean atDesiredPosition() {
        double wristPosition = mWristMotor.getSelectedSensorPosition();
        double armPosition = mShoulderMotor1.getSelectedSensorPosition();

        if ((Math.abs(armPosition - desiredShoulderPosition.get()) < ARM_ERROR) && (Math.abs(wristPosition - desiredWristPosition()) < WRIST_ERROR)) {
            return true;
        }
        return false;
    }

    public double convertDegreesToTicksShoulder(double degrees) {
        return degrees * 2048 * 120 / 360; //2048 ticks per rotation, 120:1 gear down ratio, 360 degrees per rotation
    }

    public double convertDegreesToTicksWrist(double degrees) {
        return degrees * 2048 * 60 / 360; //2048 ticks per rotation, 60:1 gear down ratio, 360 degrees per rotation
    }

}
