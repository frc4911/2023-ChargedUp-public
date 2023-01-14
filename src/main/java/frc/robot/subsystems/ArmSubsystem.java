package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

    public enum WantedState {
        STOWED,
        CONE_LEVEL_3,
        CUBE_LEVEL_3,
        CONE_LEVEL_2,
        CUBE_LEVEL_2,
        HYBRID_CONE,
        HYBRID_CUBE,
        COLLECT_PORTAL,
        COLLECT_GROUND
    }

    public ArmSubsystem() {

        //1 is closest to robot center and the numbering moves out clockwise
        mShoulderMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_1, Constants.kCanivoreName);
        mShoulderMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_2, Constants.kCanivoreName);
        mShoulderMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_3, Constants.kCanivoreName);
        mShoulderMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOULDER_MOTOR_4, Constants.kCanivoreName);
        mWristMotor = TalonFXFactory.createDefaultTalon(Ports.WRIST_MOTOR, Constants.kCanivoreName);
    }

    public void moveArm(WantedState state) {
        switch(state) {
            case STOWED:
                
            case CONE_LEVEL_3:
            case CUBE_LEVEL_3:

            case CONE_LEVEL_2:
            case CUBE_LEVEL_2:

            case HYBRID_CONE:
            case HYBRID_CUBE:

            case COLLECT_PORTAL:

            case COLLECT_GROUND:
                break;
        }
    }

    

    public double convertDegreesToTicks(double degrees) {
        return degrees; //Can do once we get gear ratios from DESIGN
    }

    public double convertTicksToDegrees(double ticks) {
        return ticks;
    }
}
