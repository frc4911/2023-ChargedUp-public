package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Default arm command that holds the last set position. */
public final class DefaultArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public DefaultArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.checkAndCorrectCurrentPosition();
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBrakeMode();
    }
}
