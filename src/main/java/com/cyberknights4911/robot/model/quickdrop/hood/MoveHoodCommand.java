package com.cyberknights4911.robot.model.quickdrop.hood;

import com.cyberknights4911.robot.model.quickdrop.hood.HoodSubsystem.HoodPositions;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveHoodCommand extends CommandBase {

    private final HoodSubsystem mHoodSubsystem;
    private final HoodPositions mDesiredHoodPosition;
    public MoveHoodCommand(HoodSubsystem hoodSubsystem, HoodPositions desiredPosition) {

        mHoodSubsystem = hoodSubsystem;
        mDesiredHoodPosition = desiredPosition;
        addRequirements(mHoodSubsystem);
    }

    @Override
    public void initialize() {
        mHoodSubsystem.setDesiredHoodPosition(mDesiredHoodPosition);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
