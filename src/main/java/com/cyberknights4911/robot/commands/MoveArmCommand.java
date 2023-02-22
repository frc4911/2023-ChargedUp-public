package com.cyberknights4911.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;

public class MoveArmCommand extends CommandBase {

    private final ArmSubsystem mArmSubsystem;
    private final ArmPositions mDesiredArmPosition;
    public MoveArmCommand(ArmSubsystem armSubsystem, ArmPositions desiredPosition) {

        mArmSubsystem = armSubsystem;
        mDesiredArmPosition = desiredPosition;
        addRequirements(mArmSubsystem);
    }

    @Override
    public void initialize() {
        mArmSubsystem.setDesiredPosition(mDesiredArmPosition);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        mArmSubsystem.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return mArmSubsystem.wristAtDesiredPosition() && mArmSubsystem.shoulderAtDesiredPosition();
    }

    
}
