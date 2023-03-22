package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
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

            if(desiredPosition == ArmPositions.SCORE_L3 || desiredPosition == ArmPositions.COLLECT_SUBSTATION_BACK || desiredPosition == ArmPositions.COLLECT_FLOOR_BACK_CUBE || desiredPosition == ArmPositions.COLLECT_FLOOR_BACK_CONE) {
                
                armSubsystem.moveWrist(Constants.Arm.WRIST_TUCKED_ANGLE_FRONT_TO_BACK.getValue());

            } else {

                armSubsystem.moveWrist(Constants.Arm.WRIST_TUCKED_ANGLE_BACK_TO_FRONT.getValue());
            }
        } else {
            
            armSubsystem.moveWrist(desiredPosition.wristPosition);
        }
        // Always begin moving the shoulder immediately
        armSubsystem.moveShoulder(desiredPosition.shoulderPosition);
    }

    @Override
    public void execute() {
        double currentShoulderPosition = armSubsystem.getShoulderPositionDegrees();
        // If this is a tucking command, we need to move the wrist AFTER the shoulder is safe
        if (shouldTuckWrist && !isWristTucked && currentShoulderPosition > safePosition) {
            armSubsystem.moveWrist(desiredPosition.wristPosition);
            // No need to keep sending the moveWrist call.
            isWristTucked = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }
    
    /**
     * Creates a new command for moving the arm. The command defers creation of the actual
     * command until it is actually needed.
     */
    public static Command create(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        // TODO: check whether the proxying is even necessary (we check position on initialization)
        ProxyCommand command = new ProxyCommand(() -> createImmediate(armSubsystem, desiredPosition));
        command.addRequirements(armSubsystem);
        return command;
    }

    private static Command createImmediate(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition);
    }
}
