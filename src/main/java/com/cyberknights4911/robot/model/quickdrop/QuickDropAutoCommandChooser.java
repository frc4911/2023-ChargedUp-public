package com.cyberknights4911.robot.model.quickdrop;

import com.cyberknights4911.robot.auto.AutoCommandChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class QuickDropAutoCommandChooser implements AutoCommandChooser {

    @Override
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
    
}
