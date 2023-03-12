package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class MoveArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;
    private final boolean isIntermediate;

    private boolean isFinished = false;

    private MoveArmCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;

        switch(desiredPosition) {
            case INTERMEDIATE_FRONT_FROM_FRONT:
            case INTERMEDIATE_BACK_FROM_FRONT:
            case INTERMEDIATE_FRONT_FROM_BACK:
            case INTERMEDIATE_BACK_FROM_BACK:
                this.isIntermediate = true;
                break;
            default:
                this.isIntermediate = false;
                break;
        }
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.reset();
    }

    @Override
    public void execute() {
        boolean success = armSubsystem.moveWristPid(desiredPosition) &&
            armSubsystem.moveShoulderPid(desiredPosition, isIntermediate);
        if (!success) {
            // Shoulder or wrist encoder are offline. Movements may cause damage.
            armSubsystem.setBrakeMode();
            isFinished = true;
        } else if (isIntermediate) {
            isFinished = armSubsystem.isNearPosition(desiredPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
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
        boolean isDesiredPositionFront = desiredPosition.shoulderState.position < 180 ;
        boolean isCurrentPositionFront = armSubsystem.isCurrentArmFront();

        if (isDesiredPositionFront && !isCurrentPositionFront) {
            // Moving back to front
            return new SequentialCommandGroup(
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_BACK_FROM_FRONT),
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_FRONT_FROM_FRONT),
                new MoveArmCommand(armSubsystem, desiredPosition)
            );
        } else if (!isDesiredPositionFront && isCurrentPositionFront) {
            // Moving front to back
            return new SequentialCommandGroup(
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_FRONT_FROM_FRONT),
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_BACK_FROM_FRONT),
                new MoveArmCommand(armSubsystem, desiredPosition)
            );
        } else {
            return new MoveArmCommand(armSubsystem, desiredPosition);
        }
    }
}
