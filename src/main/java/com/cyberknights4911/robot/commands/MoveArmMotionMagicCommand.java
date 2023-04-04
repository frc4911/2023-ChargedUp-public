package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.DoublePreference;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public final class MoveArmMotionMagicCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;
    private final InitializedListener listener;

    private boolean shouldTuckWrist;
    private boolean isWristTucked;
    private double safePosition;

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
        safePosition = 0;

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
                    this.shouldTuckWrist = true;
                    this.safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_FRONT.getValue();
                    desiredWristPosition = Constants.Arm.WRIST_TUCKED_ANGLE_BACK_TO_FRONT.getValue();
                }
                break;
            case SCORE_L3:
            case COLLECT_SUBSTATION_BACK:
                if (currentArmPosition < 180) {
                    this.shouldTuckWrist = true;
                    this.safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_TOP.getValue();
                    desiredWristPosition = Constants.Arm.WRIST_TUCKED_ANGLE_FRONT_TO_BACK.getValue();
                } else if (currentArmPosition > 270) {
                    this.shouldTuckWrist = true;
                    this.safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_MIDDLE.getValue();
                }
                break;
            case COLLECT_FLOOR_BACK_CUBE:
            case COLLECT_FLOOR_BACK_CONE:
            // TODO: try combining these cases with those immediately above. See if violations are even possible.
            if (currentArmPosition < 270) {
                this.shouldTuckWrist = true;
                this.safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_BOTTOM.getValue();
            }
                break;
            default:
                this.shouldTuckWrist = false;
                break;
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
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }

    public ArmPositions getDesiredPosition() {
        return desiredPosition;
    }

    /**
     * Creates a command that terminates when the arm movement is complete. This is useful for
     * building command sequences that need to wait for the arm to reach a requested postion since
     * arm commands never actually finish themselves.
     */
    public CommandBase getMovementFinishedCommand() {
        // Use proxy to defer 
        return new ProxyCommand(() -> {
            return new CommandBase() {
                @Override
                public boolean isFinished() {
                    return armSubsystem.isCurrentMotionFinished();
                }
            };
        });
    }

    /** Used in tuning mode to move to the desired positions after modifying them. */
    private void tuningModeAdjust() {
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
                currentCommand.tuningModeAdjust();
            });
        }

        private CommandBase decrease(MoveArmMotionMagicCommand currentCommand, DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() - 1);
                currentCommand.tuningModeAdjust();
            });
        }

        @Override
        public void onInitialize(MoveArmMotionMagicCommand armCommand) {
            currentCommand = armCommand;
        }
    }
}
