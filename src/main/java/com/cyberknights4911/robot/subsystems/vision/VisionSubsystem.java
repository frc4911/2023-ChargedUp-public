package com.cyberknights4911.robot.subsystems.vision;

import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.vision.AprilTagCoprocessorMessenger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for handling vision.
 */
public class VisionSubsystem extends SubsystemBase {

    private AprilTagCoprocessorMessenger messenger;

    public VisionSubsystem() {
        messenger = new AprilTagCoprocessorMessenger(NetworkTableInstance.getDefault());
        messenger.subscribe();
    } 
}
