package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class MoveArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions desiredPosition;

    public MoveArmCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.reset();
    }

    @Override
    public void execute() {
        armSubsystem.moveWristPid(desiredPosition);
        armSubsystem.moveShoulderPid(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
