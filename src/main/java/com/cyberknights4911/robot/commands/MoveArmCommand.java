package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class MoveArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;
    private final boolean isIntermediate;

    private boolean isFinished = false;

    private MoveArmCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        this(armSubsystem, desiredPosition, false);
    }

    private MoveArmCommand(
        ArmSubsystem armSubsystem, ArmPositions desiredPosition, boolean isIntermediate
    ) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;
        this.isIntermediate = isIntermediate;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.reset();
    }

    @Override
    public void execute() {
        boolean success = armSubsystem.moveWristPid(desiredPosition) &&
            armSubsystem.moveShoulderPid(desiredPosition);
        if (!success) {
            // Shoulder or wrist encoder are offline. Movements may cause damage.
            armSubsystem.setBrakeMode();
            isFinished = true;
        } else if (isIntermediate) {
            isFinished = armSubsystem.atDesiredPosition(desiredPosition);
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

    public static Command create(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        boolean isDesiredPositionFront = desiredPosition.shoulderPosition < 180 ;
        boolean isCurrentArmFront = armSubsystem.isCurrentArmFront();

        if (isDesiredPositionFront && !isCurrentArmFront) {
            return new SequentialCommandGroup(
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_BACK, true),
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_FRONT, true),
                new MoveArmCommand(armSubsystem, desiredPosition)
            );
        } else if (!isDesiredPositionFront && isCurrentArmFront) {
            return new SequentialCommandGroup(
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_FRONT, true),
                new MoveArmCommand(armSubsystem, ArmPositions.INTERMEDIATE_BACK, true),
                new MoveArmCommand(armSubsystem, desiredPosition)
            );
        } else {
            return new MoveArmCommand(armSubsystem, desiredPosition);
        }
    }
}
