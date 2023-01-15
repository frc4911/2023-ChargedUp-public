package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
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
        mShoulderMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_1, Constants.kCanivoreName);
        mShoulderMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_2, Constants.kCanivoreName);
        mShoulderMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_3, Constants.kCanivoreName);
        mShoulderMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_4, Constants.kCanivoreName);
        mWristMotor = TalonFXFactory.createDefaultTalon(Ports.WRIST_MOTOR, Constants.kCanivoreName);
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

        mShoulderMotor1.configAllSettings(ShoulderConfiguration);
        mShoulderMotor2.configAllSettings(ShoulderConfiguration);
        mShoulderMotor3.configAllSettings(ShoulderConfiguration);
        mShoulderMotor4.configAllSettings(ShoulderConfiguration);

        mShoulderMotor3.setInverted(true);
        mShoulderMotor4.setInverted(true);

        //May want to use setStatusFramePeriod to be lower and make motors followers if having CAN utilization issues
        mShoulderMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        //WRIST CONFIGURATION
        TalonFXConfiguration WristConfiguration = new TalonFXConfiguration();
        WristConfiguration.supplyCurrLimit.currentLimit = 20.0;
        WristConfiguration.supplyCurrLimit.enable = true;
        WristConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        WristConfiguration.slot0.kP = 0.25; //Default PID values no rhyme or reason
        WristConfiguration.slot0.kI = 0.0;
        WristConfiguration.slot0.kD = 0.0;

        mWristMotor.configAllSettings(ShoulderConfiguration);


    }

    public void setShoulderPositionDesiredPosition(ArmPositions desiredPosition) {
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
        double falconTicks = convertDegreesToTicksWrist(desiredShoulderPosition.get()%180);
        mWristMotor.set(ControlMode.Position, falconTicks);
    }

    public double convertDegreesToTicksShoulder(double degrees) {
        return degrees; //Can do once we get gear ratios from DESIGN
    }

    public double convertDegreesToTicksWrist(double degrees) {
        return degrees;//Can do calculations once we get gear ratios from DESIGN
    }

}
