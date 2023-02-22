package com.cyberknights4911.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_GEAR_RATIO = 120.0;
    public static final double WRIST_GEAR_RATIO = 60.0;
    public static final int TICKS_PER_REVOLUTION = 2048;
    public static final int DEGREES_PER_REVOLUTION = 360;
    //In ticks can be changed to be in degrees
    //Makes it run much faster because it does not need to be precise
    private static final double ARM_ERROR = 5000;
    private static final double WRIST_ERROR = 5000;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private ArmPositions desiredPosition = ArmPositions.STOWED;

    public ArmSubsystem(ArmIO armIO) {
        super();
        this.armIO = armIO;
    }

    public void setDesiredPosition(ArmPositions desiredPosition) {
        this.desiredPosition = desiredPosition;
        moveWrist(desiredPosition);
        moveShoulder(desiredPosition);
    }

    public void setBrakeMode() {
        armIO.setBrakeMode();
    }

    private void moveShoulder(ArmPositions desiredArmPosition) {
        double falconTicks = convertDegreesToTicksShoulder(desiredArmPosition.getShoulderPosition());
        armIO.setShoulderPosition(falconTicks);
    }

    private void moveWrist(ArmPositions desiredArmPosition) {
        double falconTicks = convertDegreesToTicksWrist(desiredArmPosition.getWristPosition());
        armIO.setWristPosition(falconTicks);
    }

    //Calculated in ticks at the moment
    public boolean wristAtDesiredPosition() {
        double wristPosition = armIO.getWristPosition();
        double desiredWristPosition = convertDegreesToTicksShoulder(desiredPosition.getWristPosition());
        if (Math.abs(wristPosition - desiredWristPosition) < WRIST_ERROR) {
            return true;
        }
        return false;
    }

    //Calculated in ticks at the moment
    public boolean shoulderAtDesiredPosition() {
        double shoulderPosition = armIO.getShoulderPosition();
        if (Math.abs(shoulderPosition - convertDegreesToTicksShoulder(desiredPosition.getShoulderPosition())) < ARM_ERROR) {
            return true;
        }
        return false;
    }

    //Check if the robot will be too tall
    //Avoid between 70-210 degrees
    public boolean checkForHeightViolation() {
        double shoulderPosition = convertTicksToDegreesShoulder(armIO.getShoulderPosition());
        if (shoulderPosition <= 210 && shoulderPosition >= 70 ) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

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

    private double convertTicksToDegreesShoulder(double degrees) {
        return degrees * DEGREES_PER_REVOLUTION / TICKS_PER_REVOLUTION / SHOULDER_GEAR_RATIO;
    }

    private double convertDegreesToTicksWrist(double degrees) {
        return degrees * TICKS_PER_REVOLUTION * WRIST_GEAR_RATIO / DEGREES_PER_REVOLUTION;
    }

}
