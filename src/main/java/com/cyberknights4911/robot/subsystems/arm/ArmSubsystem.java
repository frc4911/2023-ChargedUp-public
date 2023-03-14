package com.cyberknights4911.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Constants.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_GEAR_RATIO = 120.0;
    public static final double WRIST_GEAR_RATIO = 60.0;
    public static final double TICKS_PER_REVOLUTION = 4096;
    public static final double DEGREES_PER_REVOLUTION = 360;
    private static final double SHOULDER_ERROR_DEGREES = 5.0;
    private static final double WRIST_ERROR_DEGREES = 2.0;

    private final ArmIO armIO;
    private final ProfiledPIDController shoulderController;
    private final ProfiledPIDController wristController;
    private final ArmFeedforward shoulderFeedforward;
    private final ArmFeedforward wristFeedforward;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO armIO) {
        super();
        this.armIO = armIO;
        shoulderController = new ProfiledPIDController(
            Arm.SHOULDER_P,
            Arm.SHOULDER_I,
            Arm.SHOULDER_D,
            new TrapezoidProfile.Constraints(Arm.SHOULDER_VELOCITY, Arm.SHOULDER_ACCELERATION)
        );
        shoulderController.setTolerance(Arm.SHOULDER_TOLERANCE);
        wristController = new ProfiledPIDController(
            Arm.WRIST_P,
            Arm.WRIST_I,
            Arm.WRIST_D,
            new TrapezoidProfile.Constraints(Arm.WRIST_VELOCITY, Arm.WRIST_ACCELERATION)
        );
        wristController.setTolerance(Arm.WRIST_TOLERANCE);
        shoulderFeedforward = new ArmFeedforward(Arm.SHOULDER_S, Arm.SHOULDER_G, Arm.SHOULDER_V);
        wristFeedforward = new ArmFeedforward(Arm.WRIST_S, Arm.WRIST_G, Arm.WRIST_V);

        reset();
    }

    public void reset() {
        shoulderController.reset(armIO.getShoulderEncoderDegrees());
        wristController.reset(armIO.getWristEncoderDegrees());
    }

    public void setBrakeMode() {
        setShoulderBrakeMode();
        setWristBrakeMode();
    }

    public void setShoulderBrakeMode() {
        armIO.setShoulderOutput(0);
        armIO.setShoulderBrakeMode();
    }

    public void setWristBrakeMode() {
        armIO.setWristOutput(0);
        armIO.setWristBrakeMode();
    }

    public boolean moveShoulderPid(ArmPositions desiredArmPosition, boolean isIntermediate) {

        if (!armIO.isShoulderEncoderConnected()) {
            System.out.println("ERROR: shoulder encoder offline.");
            return false;
        }

        shoulderController.setGoal(desiredArmPosition.shoulderState);
        double profiledPidValue = shoulderController.calculate(armIO.getShoulderEncoderDegrees());
        
        // TODO use the feedforward
        armIO.setShoulderOutput(profiledPidValue);
        return true;
    }

    public boolean moveWristPid(ArmPositions desiredArmPosition) {

        if (!armIO.isWristEncoderConnected()) {
            System.out.println("ERROR: wrist encoder offline.");
            return false;
        }
        wristController.setGoal(desiredArmPosition.wristState);
        double profiledPidValue = wristController.calculate(armIO.getWristEncoderDegrees());
        
        armIO.setWristOutput(profiledPidValue);
        return true;
    }

    public boolean moveShoulder(ArmPositions desiredArmPosition) {
        armIO.setShoulderPosition(convertDegreesToCtreTicks(desiredArmPosition.shoulderState.position));
        return true;
    }

    public boolean moveWrist(ArmPositions desiredArmPosition) {
        armIO.setWristPosition(convertDegreesToCtreTicks(desiredArmPosition.wristState.position));
        return true;
    }

    public boolean isCurrentArmFront() {
        return armIO.getShoulderEncoderDegrees() < 180;
    }

    public boolean isNearPosition(ArmPositions armPosition) {
        double wristPosition = armIO.getWristEncoderDegrees();
        double shoulderPosition = armIO.getShoulderEncoderDegrees();
        // System.out.println("Wrist is off by: " + Math.abs(wristPosition - armPosition.wristPosition));
        // System.out.println("Shoulder is off by: " + Math.abs(shoulderPosition - armPosition.shoulderPosition));
        return Math.abs(wristPosition - armPosition.wristState.position) < WRIST_ERROR_DEGREES &&
            Math.abs(shoulderPosition - armPosition.shoulderState.position) < SHOULDER_ERROR_DEGREES;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
        SmartDashboard.putNumber("WRIST encoder", armIO.getWristEncoderDegrees());
        SmartDashboard.putNumber("SHOULDER encoder", armIO.getShoulderEncoderDegrees());

    }

    public double getShoulderPositionDegrees() {
        return armIO.getShoulderEncoderDegrees();
    }

    public static double convertDegreesToCtreTicks(double degrees) {
        return degrees * TICKS_PER_REVOLUTION / DEGREES_PER_REVOLUTION;
    }

    public static double convertCtreTicksToDegrees(double ticks) {
        return ticks / TICKS_PER_REVOLUTION * DEGREES_PER_REVOLUTION;
    }

}
