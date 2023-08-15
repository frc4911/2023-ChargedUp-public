package com.cyberknights4911.robot.model.deadeye;

import com.cyberknights4911.robot.auto.AutoCommandHandler;

public final class DeadEyeAutoCommandHandler implements AutoCommandHandler {

    @Override
    public void startCurrentAutonomousCommand() {
        System.out.println("No auto command to start");
    }

    @Override
    public void stopCurrentAutonomousCommand() {
        System.out.println("No auto command to stop");
    }
}
