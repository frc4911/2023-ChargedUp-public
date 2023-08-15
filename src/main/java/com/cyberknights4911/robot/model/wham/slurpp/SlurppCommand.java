package com.cyberknights4911.robot.model.wham.slurpp;

import com.cyberknights4911.robot.model.wham.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SlurppCommand extends CommandBase {

    private final SlurppSubsystem mSlurppSubsystem;
    private final ArmSubsystem mArmSubsystem;
    private final double mPercentOutput;
    private final boolean mStopOnEnd;

    public SlurppCommand(SlurppSubsystem slurppSubsystem, double percentOutput, ArmSubsystem armSubsystem, boolean stopOnEnd) {

        mStopOnEnd = stopOnEnd;
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
    public void end(boolean interrupted) {
        if (mStopOnEnd)  {
            mSlurppSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return !mStopOnEnd;
    }
    
}
