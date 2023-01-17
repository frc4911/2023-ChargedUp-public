package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.HoodPositions;

public class MoveHoodCommand extends CommandBase {

    private final HoodSubsystem mHoodSubsystem;

    public MoveHoodCommand(HoodSubsystem hoodSubsystem) {

        mHoodSubsystem = hoodSubsystem;
        addRequirements(mHoodSubsystem);
    }

    @Override
    public void initialize() {
        mHoodSubsystem.setDesiredHoodPosition(HoodPositions.H1);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        mHoodSubsystem.setDesiredHoodPosition(HoodPositions.STOWED);

    }
    
}
