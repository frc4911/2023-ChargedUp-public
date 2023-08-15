package com.cyberknights4911.robot.model.quickdrop.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class CollectorCamera extends SubsystemBase  {

    public CollectorCamera() {
        CameraServer.startAutomaticCapture();
    }
}
