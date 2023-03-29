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
                }
                break;
            case SCORE_L3:
            case COLLECT_SUBSTATION_BACK:
                if (currentArmPosition < 180) {
                    this.shouldTuckWrist = true;
                    this.safePosition = Constants.Arm.SHOULDER_SAFE_ANGLE_BACK_TOP.getValue();
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

        // If this is a tucking command, move to the tuck position first
        if (shouldTuckWrist) {

            if(desiredPosition == ArmPositions.SCORE_L3 || desiredPosition == ArmPositions.COLLECT_SUBSTATION_BACK || desiredPosition == ArmPositions.COLLECT_FLOOR_BACK_CUBE || desiredPosition == ArmPositions.COLLECT_FLOOR_BACK_CONE) {
                
                armSubsystem.moveWrist(Constants.Arm.WRIST_TUCKED_ANGLE_FRONT_TO_BACK.getValue());

            } else {

                armSubsystem.moveWrist(Constants.Arm.WRIST_TUCKED_ANGLE_BACK_TO_FRONT.getValue());
            }
        } else {
            armSubsystem.moveWrist(desiredPosition.wristPosition.getValue());
        }
        // Always begin moving the shoulder immediately
        armSubsystem.moveShoulder(desiredPosition.shoulderPosition.getValue());

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
                    return MoveArmMotionMagicCommand.this.isFinished()
                        || MoveArmMotionMagicCommand.this.isScheduled()
                        && armSubsystem.isCurrentMotionFinished();
                }
            };
        });
    }

    /**
     * Creates a new command for moving the arm.
     */
    public static MoveArmMotionMagicCommand create(
            ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition, (command) -> {});
    }
    /**
     * Creates a new command for moving the arm.
     */
    private static MoveArmMotionMagicCommand create(
            ArmSubsystem armSubsystem, ArmPositions desiredPosition, InitializedListener listener) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition, listener);
    }

    public static void setupTestMode(ArmSubsystem armSubsystem) {
        TestMode testMode = new TestMode();
        ShuffleboardTab tab = Shuffleboard.getTab("ARM POSITIONS");

        tab.add("STOWED", create(armSubsystem, ArmPositions.STOWED, testMode));
        tab.add("SCORE_L2", create(armSubsystem, ArmPositions.SCORE_L2, testMode));
        tab.add("COLLECT_SUBSTATION_FRONT", create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_FRONT, testMode));
        tab.add("COLLECT_FLOOR_FRONT_CONE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE, testMode));
        tab.add("COLLECT_FLOOR_FRONT_CUBE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CUBE, testMode));
        tab.add("SCORE_L3", create(armSubsystem, ArmPositions.SCORE_L3, testMode));
        tab.add("COLLECT_SUBSTATION_BACK", create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK, testMode));
        tab.add("COLLECT_FLOOR_BACK_CUBE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CUBE, testMode));
        tab.add("COLLECT_FLOOR_BACK_CONE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE, testMode));

        tab.add("SHOULDER+", testMode.increaseShoulder());
        tab.add("SHOULDER-", testMode.decreaseShoulder());
        tab.add("WRIST+", testMode.increaseWrist());
        tab.add("WRIST-", testMode.decreaseWrist());

        tab.add("RE-RUN", testMode.rerun());

        tab.add("STOW & RE-RUN", testMode.stowAndRerun(armSubsystem));
    }

    private interface InitializedListener {
        void onInitialize(MoveArmMotionMagicCommand armCommand);
    }

    private static class TestMode implements InitializedListener {
        private MoveArmMotionMagicCommand currentCommand = null;

        CommandBase increaseShoulder() {
            return new ProxyCommand(
                () -> increase(currentCommand.desiredPosition.shoulderPosition));
        }

        CommandBase decreaseShoulder() {
            return new ProxyCommand(
                () -> decrease(currentCommand.desiredPosition.shoulderPosition));
        }

        CommandBase increaseWrist() {
            return new ProxyCommand(
                () -> increase(currentCommand.desiredPosition.wristPosition));
        }

        CommandBase decreaseWrist() {
            return new ProxyCommand(
                () -> decrease(currentCommand.desiredPosition.wristPosition));
        }

        private CommandBase increase(DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() + 1);
            });
        }

        private CommandBase decrease(DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() - 1);
            });
        }

        CommandBase rerun() {
            return new ProxyCommand(() -> currentCommand);
        }

        CommandBase stowAndRerun(ArmSubsystem armSubsystem) {
            return create(armSubsystem, ArmPositions.STOWED)
                .getMovementFinishedCommand()
                .andThen(currentCommand);
        }

        @Override
        public void onInitialize(MoveArmMotionMagicCommand armCommand) {
            currentCommand = armCommand;
            
        }
    }
}
