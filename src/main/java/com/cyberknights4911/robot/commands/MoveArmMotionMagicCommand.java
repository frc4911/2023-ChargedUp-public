package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class MoveArmMotionMagicCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;

    private boolean shouldTuckwrist;
    private double safePosition;
    private boolean isShoulderSafe;

    private MoveArmMotionMagicCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;


        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.reset();

        shouldTuckwrist = false;
        safePosition = 0;

        double currentArmPosition = armSubsystem.getShoulderPositionDegrees();
        
        switch (desiredPosition) {
            case STOWED:
            case SCORE_L2:
            case COLLECT_SUBSTATION_FRONT:
            case COLLECT_FLOOR_FRONT_CONE:
            case COLLECT_FLOOR_FRONT_CUBE:
                if (currentArmPosition > 180) {
                    this.shouldTuckwrist = true;
                    this.safePosition = ArmPositions.INTERMEDIATE_FRONT_FROM_BACK.shoulderState.position;
                }
                break;
            case SCORE_L3:
            case COLLECT_SUBSTATION_BACK:
                if (currentArmPosition < 180) {
                    this.shouldTuckwrist = true;
                    this.safePosition = ArmPositions.INTERMEDIATE_BACK_FROM_FRONT.shoulderState.position;
                }
                break;
            case COLLECT_FLOOR_BACK_CUBE:
            case COLLECT_FLOOR_BACK_CONE:
                break;
            default:
                this.shouldTuckwrist = false;
                break;
        }

        if (shouldTuckwrist) {
            armSubsystem.moveWrist(ArmPositions.INTERMEDIATE_BACK_FROM_BACK);
        } else {
            armSubsystem.moveWrist(desiredPosition);
        }
        armSubsystem.moveShoulder(desiredPosition);
    }

    @Override
    public void execute() {
        if (shouldTuckwrist && armSubsystem.getShoulderPositionDegrees() > safePosition) {
            armSubsystem.moveWrist(desiredPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }
    
    /**
     * Creates a new command for moving the arm. Depending on current arm position, this may be a
     * sequence of commands or a single move command. The command defers creation of the actual
     * command or command sequence until it is actually needed.
     */
    public static Command create(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        ProxyCommand command = new ProxyCommand(() -> createImmediate(armSubsystem, desiredPosition));
        command.addRequirements(armSubsystem);
        return command;
    }

    private static Command createImmediate(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition);
    }
}
