package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.DoublePreference;
import com.cyberknights4911.robot.subsystems.arm.ArmIO;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public final class MoveArmMotionMagicCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;
    private final InitializedListener listener;
    
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

    private MoveArmMotionMagicCommand(
            ArmSubsystem armSubsystem, ArmPositions desiredPosition, InitializedListener listener) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;
        this.listener = listener;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.reset();

        shouldTuckWrist = false;
        isWristTucked = false;
        isMovingFinal = true;
        safePosition = 0;
        thresholdLoops = 0;
        retryStartTime = Timer.getFPGATimestamp();

        double currentArmPosition = armSubsystem.getShoulderPositionDegrees();
        double desiredWristPosition = desiredPosition.wristPosition.getValue();
        
        switch (desiredPosition) {
            case STOWED:
            case SCORE_L2:
            case COLLECT_SINGLE_SUBSTATION_FRONT:
            case COLLECT_SUBSTATION_FRONT:
            case COLLECT_FLOOR_FRONT_CONE:
            case COLLECT_FLOOR_FRONT_CUBE:
                if (currentArmPosition > 180) {
                    shouldTuckWrist = true;
                    safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_FRONT.getValue();
                    desiredWristPosition = Constants.Arm.WRIST_TUCKED_ANGLE_BACK_TO_FRONT.getValue();
                }
                break;
            case SCORE_L3:
            case COLLECT_SUBSTATION_BACK:
                if (currentArmPosition < 180) {
                    shouldTuckWrist = true;
                    safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_TOP.getValue();
                    desiredWristPosition = Constants.Arm.WRIST_TUCKED_ANGLE_FRONT_TO_BACK.getValue();
                } else if (currentArmPosition > 270) {
                    shouldTuckWrist = true;
                    safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_MIDDLE.getValue();
                }
                break;
            case COLLECT_FLOOR_BACK_CUBE:
            case COLLECT_FLOOR_BACK_CONE:
            // TODO: try combining these cases with those immediately above. See if violations are even possible.
            if (currentArmPosition < 270) {
                shouldTuckWrist = true;
                safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_BOTTOM.getValue();
            }
                break;
            default:
                shouldTuckWrist = false;
                break;
        }

        if (shouldTuckWrist) {
            isMovingFinal = false;
        }

        armSubsystem.moveShoulder(desiredPosition.shoulderPosition.getValue());
        armSubsystem.moveWrist(desiredWristPosition);

        listener.onInitialize(this);
    }

    @Override
    public void execute() {
        double currentShoulderPosition = armSubsystem.getShoulderPositionDegrees();
        // If this is a tucking command, we need to move the wrist AFTER the shoulder is safe
        if (shouldTuckWrist && !isWristTucked && currentShoulderPosition > safePosition) {
            armSubsystem.moveWrist(desiredPosition.wristPosition.getValue());
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
        double shoulderError = Math.abs(armSubsystem.getShoulderTrajectoryPosition() - ArmSubsystem.convertDegreesToCtreTicks(desiredPosition.shoulderPosition.getValue()));
        double wristError = Math.abs(armSubsystem.getWristTrajectoryPosition() - ArmSubsystem.convertDegreesToCtreTicks(desiredPosition.wristPosition.getValue()));
        System.out.println("shoulder error: " + shoulderError);
        System.out.println("wrist error: " + wristError);
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

    public ArmPositions getDesiredPosition() {
        return desiredPosition;
    }

    /** Used in tuning mode to move to the desired positions after modifying them. */
    private void rerunMoveToPosition() {
        System.out.println("rerunMoveToPosition");
        armSubsystem.moveWrist(desiredPosition.wristPosition.getValue());
        armSubsystem.moveShoulder(desiredPosition.shoulderPosition.getValue());
    }

    /**
     * Creates a new command for moving the arm.
     */
    public static MoveArmMotionMagicCommand create(
            ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition, LISTENER);
    }

    private static final InitializedListener LISTENER = Constants.Arm.IS_TUNING_ENABLED ?
       new TestMode() : (command) -> {};

    private interface InitializedListener {
        void onInitialize(MoveArmMotionMagicCommand armCommand);
    }

    private static class TestMode implements InitializedListener {
        private MoveArmMotionMagicCommand currentCommand = null;

        private TestMode() {
            ShuffleboardTab tab = Shuffleboard.getTab("ARM TUNING");
            tab.add("SHOULDER+", increaseShoulder());
            tab.add("SHOULDER-", decreaseShoulder());
            tab.add("WRIST+", increaseWrist());
            tab.add("WRIST-", decreaseWrist());
        }

        CommandBase increaseShoulder() {
            return new ProxyCommand(
                () -> increase(currentCommand, currentCommand.desiredPosition.shoulderPosition));
        }

        CommandBase decreaseShoulder() {
            return new ProxyCommand(
                () -> decrease(currentCommand, currentCommand.desiredPosition.shoulderPosition));
        }

        CommandBase increaseWrist() {
            return new ProxyCommand(
                () -> increase(currentCommand, currentCommand.desiredPosition.wristPosition));
        }

        CommandBase decreaseWrist() {
            return new ProxyCommand(
                () -> decrease(currentCommand, currentCommand.desiredPosition.wristPosition));
        }

        private CommandBase increase(MoveArmMotionMagicCommand currentCommand, DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() + 1);
                currentCommand.rerunMoveToPosition();
            });
        }

        private CommandBase decrease(MoveArmMotionMagicCommand currentCommand, DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() - 1);
                currentCommand.rerunMoveToPosition();
            });
        }

        @Override
        public void onInitialize(MoveArmMotionMagicCommand armCommand) {
            currentCommand = armCommand;
        }
    }
}
