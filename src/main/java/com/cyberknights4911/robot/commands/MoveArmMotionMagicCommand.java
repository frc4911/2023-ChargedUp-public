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
    private final boolean isIntermediate;

    private boolean isFinished = false;

    private MoveArmMotionMagicCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
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
        armSubsystem.moveWrist(desiredPosition);
        armSubsystem.moveShoulder(desiredPosition, isIntermediate);
    }

    @Override
    public void execute() {
        // TODO: do intermediate stuff
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
        return new MoveArmMotionMagicCommand(armSubsystem, desiredPosition);
    }
}
