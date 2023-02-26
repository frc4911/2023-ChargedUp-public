package com.cyberknights4911.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.cyberknights4911.robot.commands.DefaultArmCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_GEAR_RATIO = 120.0;
    public static final double WRIST_GEAR_RATIO = 60.0;
    public static final int TICKS_PER_REVOLUTION = 2048;
    public static final int DEGREES_PER_REVOLUTION = 360;
    private static final double SHOULDER_ERROR_DEGREES = 1.0;
    private static final double WRIST_ERROR_DEGREES = 1.0;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private ArmPositions currentPosition = ArmPositions.STOWED;

    public ArmSubsystem(ArmIO armIO) {
        super();
        this.armIO = armIO;
        setDefaultCommand(new DefaultArmCommand(this));
    }

    public void setCurrentPosition(ArmPositions desiredPosition) {
        this.currentPosition = desiredPosition;
        moveWrist(desiredPosition);
        moveShoulder(desiredPosition);
    }

    public void setBrakeMode() {
        armIO.setShoulderBrakeMode();
        armIO.setWristBrakeMode();
    }

    public void setShoulderBrakeMode() {
        armIO.setShoulderOutput(0);
        armIO.setShoulderBrakeMode();
    }

    public void setWristBrakeMode() {
        armIO.setWristOutput(0);
        armIO.setWristBrakeMode();
    }

    private void moveShoulder(ArmPositions desiredArmPosition) {
        System.out.println("MOVE SHOULDER");
        System.out.println("  DESIRED: " + desiredArmPosition.shoulderPosition);
        System.out.println("  CURRENT: " + armIO.getShoulderEncoderDegrees());
        double deltaDegrees = desiredArmPosition.shoulderPosition - armIO.getShoulderEncoderDegrees();
        if (deltaDegrees > 0) {
            System.out.println("POSITIVE");
            // Positive for away from front stowed position
            armIO.setShoulderOutput(.1);
        } else {
            System.out.println("NEGATIVE");
            // Negative for toward front stowed position
            armIO.setShoulderOutput(-0.1);
        }
    }

    private void moveWrist(ArmPositions desiredArmPosition) {
        System.out.println("MOVE WRIST");
        System.out.println("  DESIRED: " + desiredArmPosition.wristPosition);
        System.out.println("  CURRENT: " + armIO.getWristEncoderDegrees());
        if (true) {
            return;
        }
        double deltaDegrees = desiredArmPosition.wristPosition - armIO.getWristEncoderDegrees();
        if (deltaDegrees > 0) {
            System.out.println("POSITIVE");
            // Positive for toward slurpp motor
            armIO.setWristOutput(.1);
        } else {
            System.out.println("NEGATIVE");
            // Negative for away from slurpp motor
            armIO.setWristOutput(-0.1);
        }
    }

    public void checkAndCorrectCurrentPosition() {
        moveWrist(currentPosition);
        moveShoulder(currentPosition);
    }

    //Calculated in ticks at the moment
    public boolean wristAtDesiredPosition(ArmPositions armPosition) {
        return true;
        // double wristPosition = armIO.getWristEncoderDegrees();
        // return Math.abs(wristPosition - armPosition.wristPosition) < WRIST_ERROR_DEGREES;
    }

    //Calculated in ticks at the moment
    public boolean shoulderAtDesiredPosition(ArmPositions armPosition) {
        double shoulderPosition = armIO.getShoulderEncoderDegrees();
        return Math.abs(shoulderPosition - armPosition.shoulderPosition) < SHOULDER_ERROR_DEGREES;
    }

    //Check if the robot will be too tall
    //Avoid between 70-210 degrees
    public boolean checkForHeightViolation() {
        double shoulderPosition = convertTicksToDegreesShoulder(armIO.getShoulderEncoderDegrees());
        if (shoulderPosition <= 210 && shoulderPosition >= 70 ) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
        SmartDashboard.putNumber("WRIST encoder", armIO.getWristEncoderDegrees());
        SmartDashboard.putNumber("SHOULDER encoder", armIO.getShoulderEncoderDegrees());


        //Override wrist position to avoid being too tall

        // if (checkForHeightViolation()) {
        //     armIO.setWristPosition(0);
        // } else {
        //     moveWrist(desiredPosition);
        // }
        //TODO:Add back in

    }

    private double convertDegreesToTicksShoulder(double degrees) {
        return degrees * TICKS_PER_REVOLUTION * SHOULDER_GEAR_RATIO / DEGREES_PER_REVOLUTION;
    }

    private double convertTicksToDegreesShoulder(double ticks) {
        return ticks * DEGREES_PER_REVOLUTION / TICKS_PER_REVOLUTION / SHOULDER_GEAR_RATIO;
    }

    private double convertDegreesToTicksWrist(double degrees) {
        return degrees * TICKS_PER_REVOLUTION * WRIST_GEAR_RATIO / DEGREES_PER_REVOLUTION;
    }

}
