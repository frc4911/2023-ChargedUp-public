package com.cyberknights4911.robot.model.wham.arm;

import com.cyberknights4911.robot.constants.DoublePreference;
import com.cyberknights4911.robot.model.wham.WhamConstants;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import com.cyberknights4911.robot.model.wham.slurpp.CollectConfig.CollectSide;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

final class MoveArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;
    private final SlurppSubsystem slurppSubsystem;

    private final double shoulderErrorThreshold = 1.0;
    private final double wristErrorThreshold = 1.0;
    private final double retryAfter = .5;

    private final int loopsToSettle = 40; // how many loops sensor must be close-enough
    private int thresholdLoops = 0;

    private boolean shouldTuckWrist;
    private boolean isWristTucked;
    private boolean isMovingFinal;
    private double safePosition;
    private double retryStartTime;

    MoveArmCommand(ArmSubsystem armSubsystem, SlurppSubsystem slurpSubsystem,
            ArmPositions desiredPosition) {
        this.armSubsystem = armSubsystem;
        this.slurppSubsystem = slurpSubsystem;
        this.desiredPosition = desiredPosition;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        shouldTuckWrist = false;
        isWristTucked = false;
        isMovingFinal = true;
        safePosition = 0;
        thresholdLoops = 0;
        retryStartTime = Timer.getFPGATimestamp();

        double currentArmPosition = armSubsystem.getShoulderPositionDegrees();
        double desiredWristPosition = getWristPosition().getValue();
        
        switch (desiredPosition) {
            case STOWED:
            case SCORE_L2:
            case COLLECT_SUBSTATION_FRONT:
            case COLLECT_FLOOR_FRONT_CONE:
                if (currentArmPosition > 180) {
                    shouldTuckWrist = true;
                    safePosition = WhamConstants.Arm.SHOULDER_SAFE_ANGLE_FRONT.getValue();
                    desiredWristPosition = WhamConstants.Arm.WRIST_TUCKED_ANGLE_BACK_TO_FRONT.getValue();
                }
                break;
            case SCORE_L3:
            case COLLECT_SUBSTATION_BACK:
                if (currentArmPosition < 180) {
                    shouldTuckWrist = true;
                    safePosition = WhamConstants.Arm.SHOULDER_SAFE_ANGLE_BACK_TOP.getValue();
                    desiredWristPosition = WhamConstants.Arm.WRIST_TUCKED_ANGLE_FRONT_TO_BACK.getValue();
                } else if (currentArmPosition > 270) {
                    shouldTuckWrist = true;
                    safePosition = WhamConstants.Arm.SHOULDER_SAFE_ANGLE_BACK_MIDDLE.getValue();
                }
                break;
            default:
                shouldTuckWrist = false;
                break;
        }

        if (shouldTuckWrist) {
            isMovingFinal = false;
        }

        armSubsystem.moveShoulder(getShoulderPosition().getValue());
        armSubsystem.moveWrist(desiredWristPosition);

        armSubsystem.setLastCommand(this);
        updateCollectConfig();
    }

    private void updateCollectConfig() {
        switch(desiredPosition) {
            case COLLECT_SUBSTATION_FRONT:
            case COLLECT_FLOOR_FRONT_CONE:
                slurppSubsystem.setCollectSide(CollectSide.FRONT);
                break;
            case COLLECT_SUBSTATION_BACK:
                slurppSubsystem.setCollectSide(CollectSide.BACK);
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        double currentShoulderPosition = armSubsystem.getShoulderPositionDegrees();
        // If this is a tucking command, we need to move the wrist AFTER the shoulder is safe
        if (shouldTuckWrist && !isWristTucked && currentShoulderPosition > safePosition) {
            armSubsystem.moveWrist(getWristPosition().getValue());
            // No need to keep sending the moveWrist call.
            isWristTucked = true;
            isMovingFinal = true;
            // restart the retry start time
            retryStartTime = Timer.getFPGATimestamp();
        }

        if (isMovingFinal) {
            checkCurrentMotionFinished();
            if (Timer.getFPGATimestamp() - retryStartTime > retryAfter) {
                rerunMoveToPosition();
                retryStartTime = Timer.getFPGATimestamp();
            }
        }
    }

    private void checkCurrentMotionFinished() {
        double shoulderError = Math.abs(armSubsystem.getShoulderTrajectoryPosition() - ArmSubsystem.convertDegreesToCtreTicks(getShoulderPosition().getValue()));
        double wristError = Math.abs(armSubsystem.getWristTrajectoryPosition() - ArmSubsystem.convertDegreesToCtreTicks(getWristPosition().getValue()));
        // System.out.println("shoulder error: " + shoulderError);
        // System.out.println("wrist error: " + wristError);
        if (shoulderError < shoulderErrorThreshold && wristError < wristErrorThreshold) {
            thresholdLoops++;
        } else {
            thresholdLoops = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return thresholdLoops > loopsToSettle;
    }

    public DoublePreference getShoulderPosition() {
        switch (desiredPosition) {
            case STOWED: return WhamConstants.Arm.STOWED_SHOULDER;
            case COLLECT_SUBSTATION_BACK: return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER;
            case COLLECT_SUBSTATION_FRONT: return WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_SHOULDER;
            case COLLECT_FLOOR_FRONT_CONE: return WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_SHOULDER;
            case SCORE_L3: 
                switch (slurppSubsystem.getGamePiece()) {
                    case CUBE:
                        // seems weird, but this actually works better for cubes
                        return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_SHOULDER;
                    default:
                        return WhamConstants.Arm.SCORE_L3_SHOULDER;
                }
            case SCORE_L2: return WhamConstants.Arm.SCORE_L2_SHOULDER;
            default: return WhamConstants.Arm.STOWED_SHOULDER;
        }
    }

    public DoublePreference getWristPosition() {
        switch (desiredPosition) {
            case STOWED: return WhamConstants.Arm.STOWED_WRIST;
            case COLLECT_SUBSTATION_BACK: return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_WRIST;
            case COLLECT_SUBSTATION_FRONT: return WhamConstants.Arm.COLLECT_SUBSTATION_FRONT_WRIST;
            case COLLECT_FLOOR_FRONT_CONE: return WhamConstants.Arm.COLLECT_FLOOR_FRONT_CONE_WRIST;
            case SCORE_L3: 
                switch (slurppSubsystem.getGamePiece()) {
                    case CUBE:
                        // seems weird, but this actually works better for cubes
                        return WhamConstants.Arm.COLLECT_SUBSTATION_BACK_WRIST;
                    default:
                        return WhamConstants.Arm.SCORE_L3_WRIST;
                }
            case SCORE_L2: return WhamConstants.Arm.SCORE_L2_WRIST;
            default: return WhamConstants.Arm.STOWED_WRIST;
        }
    }

    /** Used in tuning mode to move to the desired positions after modifying them. */
    void rerunMoveToPosition() {
        //System.out.println("rerunMoveToPosition");
        armSubsystem.moveWrist(getWristPosition().getValue());
        armSubsystem.moveShoulder(getShoulderPosition().getValue());
    }
}
