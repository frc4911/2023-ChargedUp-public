package com.cyberknights4911.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppSubsystem;

public class SlurppCommand extends CommandBase {

    private final SlurppSubsystem mSlurppSubsystem;
    private final ArmSubsystem mArmSubsystem;
    private final double mPercentOutput;
    private boolean mCounterRelease; //This will keep the motor fighting the natural tendency for the object to fall out
    private double mPercentOutput2;
    
    public SlurppCommand(SlurppSubsystem slurppSubsystem, double percentOutput, ArmSubsystem armSubsystem, boolean counterRelease) {

        mCounterRelease = counterRelease;
        mArmSubsystem = armSubsystem;
        mSlurppSubsystem = slurppSubsystem;
        mPercentOutput = percentOutput;
        addRequirements(mSlurppSubsystem);
    }

    public SlurppCommand(SlurppSubsystem slurppSubsystem, double percentOutput, ArmSubsystem armSubsystem) {
        this(slurppSubsystem, percentOutput, armSubsystem, false);
    }

    @Override
    public void initialize() {
        if (mArmSubsystem.getShoulderPositionDegrees() > 180) {
            mSlurppSubsystem.slurpp(-mPercentOutput);
        } else {
            mSlurppSubsystem.slurpp(mPercentOutput);
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        if (false) {//Temporarily remove this because wheels are being changed
           mSlurppSubsystem.slurpp(0.03*mPercentOutput);
        } else {
            mSlurppSubsystem.stop();
        }
    }
    
}
