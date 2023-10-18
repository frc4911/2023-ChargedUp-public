package com.cyberknights4911.robot.model.wham;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.model.wham.arm.ArmSubsystem;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class BlockPartyAutos implements AutoCommandHandler {

    private final ArmSubsystem armSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LoggedDashboardChooser<Command> loggedDashboardChooser;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Pose2d> resetPose;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    
    private Command currentAutoCommand;

    

    public BlockPartyAutos(ArmSubsystem armSubsystem, SlurppSubsystem slurppSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.armSubsystem = armSubsystem;
        this.slurppSubsystem = slurppSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        

        loggedDashboardChooser = new LoggedDashboardChooser<Command>("Auto Routine");
        
        // PID constants to correct for translation error (used to create the X and Y PID controllers)
        translationConstants = new PIDConstants(4.3, 0, 0.0);

        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(2.0, 0.0, 0.0);

        poseSupplier = swerveSubsystem::getPose;
        resetPose = swerveSubsystem::setPose;
        kinematics = swerveSubsystem.getKinematics();
        outputModuleStates = swerveSubsystem::setSwerveModuleStates;
    }

    @Override
    public void startCurrentAutonomousCommand() {
        stopCurrentAutonomousCommand();
        currentAutoCommand = loggedDashboardChooser.get();
        if (currentAutoCommand != null) {
            currentAutoCommand.schedule();
        }
    }

    @Override
    public void stopCurrentAutonomousCommand() {
        if (currentAutoCommand != null) {
            currentAutoCommand.cancel();
        }
    }
}
