package com.cyberknights4911.robot.model;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.subsystems.Subsystems;

public interface RobotModel {
    public void applyDefaultCommands(Subsystems subsystems);
    public void applyButtonActions(Subsystems subsystems);
    public AutoCommandChooser geAutoCommandChooser();
}
