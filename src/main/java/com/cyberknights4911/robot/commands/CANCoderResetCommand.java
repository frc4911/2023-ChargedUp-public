package com.cyberknights4911.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppSubsystem;

public class CANCoderResetCommand extends CommandBase {

    private final ArmSubsystem mArmSubsystem;
    
    public CANCoderResetCommand(ArmSubsystem armSubsystem) {

        mArmSubsystem = armSubsystem;
        addRequirements(mArmSubsystem);
    }


    @Override
    public void initialize() {
        mArmSubsystem.resetCANCoders();
    }
        

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
