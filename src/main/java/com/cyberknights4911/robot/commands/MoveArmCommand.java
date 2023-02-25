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
        System.out.println("ARM COMMAND INIT");
        mArmSubsystem.setDesiredPosition(mDesiredArmPosition);
    }

    @Override
    public void execute() {
        System.out.println("ARM COMMAND EXECUTE");
        if (mArmSubsystem.wristAtDesiredPosition(mDesiredArmPosition)) {
            System.out.println("ARM COMMAND WRIST OKAY");
            mArmSubsystem.setWristBrakeMode();
        }

        if (mArmSubsystem.shoulderAtDesiredPosition(mDesiredArmPosition)) {
            System.out.println("ARM COMMAND SHOULDER OKAY");
            mArmSubsystem.setShoulderBrakeMode();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ARM COMMAND END");
        mArmSubsystem.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return mArmSubsystem.wristAtDesiredPosition(mDesiredArmPosition) &&
        mArmSubsystem.shoulderAtDesiredPosition(mDesiredArmPosition);
    }

    
}
