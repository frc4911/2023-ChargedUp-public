package com.cyberknights4911.robot.model.deadeye;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class DeadEyeAutoCommandChooser implements AutoCommandChooser {

    @Override
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
    
}
