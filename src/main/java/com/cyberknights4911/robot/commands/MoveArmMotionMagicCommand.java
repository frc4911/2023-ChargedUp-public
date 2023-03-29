package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public final class MoveArmMotionMagicCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;

    private boolean shouldTuckWrist;
    private boolean isWristTucked;
    private double safePosition;

    private MoveArmMotionMagicCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;

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
            // It doesn't matter which INTERMEDIATE we use, the wrist position is always the same
            armSubsystem.moveWrist(Constants.Arm.WRIST_TUCKED_ANGLE.getValue());
        } else {
            armSubsystem.moveWrist(desiredPosition.wristPosition.getValue());
        }
        // Always begin moving the shoulder immediately
        armSubsystem.moveShoulder(desiredPosition.shoulderPosition.getValue());
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
    public static MoveArmMotionMagicCommand create(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition);
    }

    public static void setupTestMode(ArmSubsystem armSubsystem) {
        ShuffleboardTab tab = Shuffleboard.getTab("ARM POSITIONS");

        tab.add("STOWED", create(armSubsystem, ArmPositions.STOWED));
        tab.add("SCORE_L2", create(armSubsystem, ArmPositions.SCORE_L2));
        tab.add("COLLECT_SUBSTATION_FRONT", create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_FRONT));
        tab.add("COLLECT_FLOOR_FRONT_CONE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE));
        tab.add("COLLECT_FLOOR_FRONT_CUBE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CUBE));
        tab.add("SCORE_L3", create(armSubsystem, ArmPositions.SCORE_L3));
        tab.add("COLLECT_SUBSTATION_BACK", create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK));
        tab.add("COLLECT_FLOOR_BACK_CUBE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CUBE));
        tab.add("COLLECT_FLOOR_BACK_CONE", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));

        tab.add("SHOULDER +", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));
        tab.add("SHOULDER -", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));
        tab.add("WRIST +", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));
        tab.add("WRIST -", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));

        tab.add("RE-RUN", create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));
    }

    private static class TestMode {
        private MoveArmMotionMagicCommand currentCommand = null;

        CommandBase cb;
        ProxyCommand pc;

        CommandBase stowAndRerun(ArmSubsystem armSubsystem) {
            return create(armSubsystem, ArmPositions.STOWED);
        }

    }
}
