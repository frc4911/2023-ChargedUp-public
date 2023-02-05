package com.cyberknights4911.robot.subsystems.drive;

import com.cyberknights4911.robot.control.ControllerBinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Common interface for Swerve Subsystems to implement. */
public interface SwerveSubsystem extends Subsystem {

    /** Get the current robot pose. */
    public Pose2d getPose();
    
    /** Set the current robot pose. */
    public void setPose(Pose2d pose);
    
    /** Get the kinematics. */
    public SwerveDriveKinematics getKinematics();

    /** Set the current state for all swerve modules. */
    public void setSwerveModuleStates(SwerveModuleState[] outputModuleStates);

    /** Create the command that should be used as the default. */
    public Command createDefaultCommand(ControllerBinding controllerBinding);

    /** Perform initialization for path following. */
    public void initForPathFollowing();
}
